"""
iot_listener.py — Módulo IoT multi-nodo v2
Lab Procesos Industriales — PUCP

CAMBIOS v2:
  - Suscripción dinámica construida desde SENSOR_NODES
  - Cada mensaje MQTT se parsea por mqtt_key, generando una lectura
    separada por cada sensor físico (t1, t2, t3, t4, flujo)
  - El state["sensors"] mantiene las 4 temperaturas individuales
    para el dashboard en vivo
  - on_reading callback guarda cada sensor por separado en PostgreSQL
"""

import asyncio
import json
import logging
import threading
import time
from datetime import datetime, timezone
from typing import Callable, Optional

logger = logging.getLogger("iot_listener")

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    logger.warning("paho-mqtt no encontrado. Instalar con: pip install paho-mqtt")


class IoTListener:
    """
    Escucha múltiples nodos MQTT y actualiza el state del servidor.

    Parámetros:
      broker       → IP del broker MQTT
      state        → dict compartido con el servidor
      port         → puerto del broker (default 1883)
      nodes_config → lista SENSOR_NODES de config.py
      on_reading   → coroutine async(node_slug, variable_slug, value)
                     llamado por cada sensor individual recibido
    """

    def __init__(
        self,
        broker: str,
        state: dict,
        port: int = 1883,
        nodes_config: list = None,
        on_reading: Optional[Callable] = None,
        keepalive: int = 60,
    ):
        """Guarda la configuración y prepara el mapa tópico→variables (ver clase)."""
        self.broker       = broker
        self.port         = port
        self.state        = state
        self.nodes_config = nodes_config or []
        self.on_reading   = on_reading
        self.keepalive    = keepalive
        self.connected    = False
        self._client      = None
        self._loop        = None
        self._last_data_time: float = 0.0

        # Mapa: tópico_completo → lista de (node_cfg, var_cfg)
        # Un mismo tópico puede tener varias variables (t1..t4 van en el mismo tópico)
        self._topic_map: dict = {}

    def seconds_since_last_data(self) -> float:
        """
        Returns:
            float: segundos desde el último dato MQTT recibido
            (inf si nunca llegó ninguno). server.py lo usa para decidir
            si los datos siguen siendo 'frescos'.
        """
        if self._last_data_time == 0.0:
            return float("inf")
        return time.monotonic() - self._last_data_time

    # ─────────────────────────────────────────
    # Arranque y parada
    # ─────────────────────────────────────────

    async def start(self):
        """
        Arranca el cliente MQTT en un hilo daemon (loop_forever):
        construye el mapa de tópicos, conecta al broker y registra callbacks.
        Si paho-mqtt no está instalado, desactiva el subsistema IoT sin fallar.
        """
        if not MQTT_AVAILABLE:
            logger.warning("paho-mqtt no instalado. IoT desactivado.")
            self.state["conexiones"]["iot"] = False
            return

        self._loop = asyncio.get_event_loop()
        self._build_topic_map()

        try:
            self._client = mqtt.Client(client_id="lab_server_v2")
            self._client.on_connect    = self._on_connect
            self._client.on_disconnect = self._on_disconnect
            self._client.on_message    = self._on_message

            logger.info(f"Conectando al broker MQTT en {self.broker}:{self.port}...")
            self._client.connect(self.broker, self.port, self.keepalive)

            thread = threading.Thread(target=self._client.loop_forever, daemon=True)
            thread.start()

        except Exception as e:
            logger.error(f"Error arrancando IoTListener: {e}")
            self.state["conexiones"]["iot"] = False

    async def stop(self):
        """Desconecta del broker y marca el subsistema IoT como caído."""
        if self._client:
            try:
                self._client.disconnect()
                self._client.loop_stop()
                logger.info("IoTListener detenido.")
            except Exception as e:
                logger.warning(f"Error deteniendo IoTListener: {e}")
        self.connected = False
        self.state["conexiones"]["iot"] = False

    # ─────────────────────────────────────────
    # Construcción del mapa de tópicos
    # ─────────────────────────────────────────

    def _build_topic_map(self):
        """
        Construye mapa {tópico: [(node_cfg, var_cfg), ...]}

        Ejemplo con el evaporador:
          "lab/evaporador/temperatura" → [
              (node_cfg, var_t1_cfg),   # mqtt_key="t1"
              (node_cfg, var_t2_cfg),   # mqtt_key="t2"
              (node_cfg, var_t3_cfg),   # mqtt_key="t3"
              (node_cfg, var_t4_cfg),   # mqtt_key="t4"
          ]
          "lab/evaporador/flujo" → [
              (node_cfg, var_flujo_cfg) # mqtt_key="value"
          ]

        Varias variables pueden compartir el mismo tópico (las 4 temperaturas
        vienen en un solo JSON), por eso el mapa guarda una lista.
        """
        self._topic_map = {}
        for node_cfg in self.nodes_config:
            prefix = node_cfg["mqtt_prefix"]
            for var_cfg in node_cfg["variables"]:
                topic = f"{prefix}/{var_cfg['mqtt_topic']}"
                if topic not in self._topic_map:
                    self._topic_map[topic] = []
                self._topic_map[topic].append((node_cfg, var_cfg))

        for topic, entries in self._topic_map.items():
            logger.info(f"Tópico registrado: {topic} → {len(entries)} variable(s)")

    # ─────────────────────────────────────────
    # Callbacks MQTT
    # ─────────────────────────────────────────

    def _on_connect(self, client, userdata, flags, rc):
        """
        Callback MQTT de conexión (rc==0 es éxito): se suscribe a todos los
        tópicos del mapa y actualiza state['conexiones']['iot'].
        """
        if rc == 0:
            logger.info(f"Conectado al broker MQTT en {self.broker}")
            self.connected = True
            self.state["conexiones"]["iot"] = True
            for topic in self._topic_map:
                client.subscribe(topic)
                logger.info(f"Suscrito a: {topic}")
        else:
            logger.error(f"Error conectando al broker MQTT. Código: {rc}")
            self.state["conexiones"]["iot"] = False

    def _on_disconnect(self, client, userdata, rc):
        """Callback MQTT de desconexión: marca IoT como caído (rc != 0 = inesperada)."""
        self.connected = False
        self.state["conexiones"]["iot"] = False
        if rc != 0:
            logger.warning(f"Desconectado del broker MQTT (código {rc})")

    def _on_message(self, client, userdata, msg):
        """
        Callback por cada mensaje MQTT (corre en el hilo de paho):
          1. Parsea el JSON del payload una sola vez.
          2. Por cada variable asociada al tópico extrae su mqtt_key,
             convierte a float y actualiza el state del dashboard.
          3. Encola on_reading() en el event loop para guardar en PostgreSQL.
        Mensajes malformados se descartan con warning (no detienen el sistema).
        """
        try:
            topic   = msg.topic
            payload = msg.payload.decode("utf-8").strip()
            logger.info(f"MQTT | topic={topic} | payload={payload}")

            if topic not in self._topic_map:
                logger.debug(f"Tópico no registrado: {topic}")
                return

            # Parsear el JSON una sola vez para todos las variables de este tópico
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                logger.warning(f"JSON inválido en {topic}: {payload}")
                return

            self._last_data_time = time.monotonic()
            self.state["sensors"]["last_update"] = datetime.now(timezone.utc).isoformat()

            # Procesar cada variable asociada a este tópico
            for node_cfg, var_cfg in self._topic_map[topic]:
                mqtt_key = var_cfg.get("mqtt_key", "value")

                # Extraer el valor de la clave correcta del JSON
                raw = data.get(mqtt_key)
                if raw is None or raw == "null":
                    logger.debug(f"Clave '{mqtt_key}' ausente o null en {topic}")
                    continue

                try:
                    value = float(raw)
                except (ValueError, TypeError):
                    logger.warning(f"No se pudo convertir a float: {mqtt_key}={raw}")
                    continue

                # Actualizar state para el dashboard en vivo
                self._update_state(node_cfg, var_cfg, value)

                # Guardar en PostgreSQL via callback async
                if self.on_reading and self._loop:
                    asyncio.run_coroutine_threadsafe(
                        self.on_reading(
                            node_slug=node_cfg["node_slug"],
                            variable_slug=var_cfg["slug"],
                            value=value
                        ),
                        self._loop
                    )

        except Exception as e:
            logger.error(f"Error procesando mensaje MQTT: {e}")

    # ─────────────────────────────────────────
    # Actualización del state para el dashboard en vivo
    # ─────────────────────────────────────────

    def _update_state(self, node_cfg: dict, var_cfg: dict, value: float):
        """
        Actualiza state["sensors"] con el valor recibido.

        Para el evaporador:
          evap_t1 → temperaturas[0]
          evap_t2 → temperaturas[1]
          evap_t3 → temperaturas[2]
          evap_t4 → temperaturas[3]
          evap_flujo → flujo

        La lógica de mapping es: el índice corresponde al orden en que
        aparece la variable dentro de node_cfg["variables"].
        """
        slug = var_cfg["slug"]

        # Para el evaporador específicamente
        if node_cfg["machine_slug"] == "evaporador":
            # Mapear slug → índice en la lista de temperaturas
            TEMP_SLUGS = ["evap_t1", "evap_t2", "evap_t3", "evap_t4"]
            if slug in TEMP_SLUGS:
                idx = TEMP_SLUGS.index(slug)
                self.state["sensors"]["temperaturas"][idx] = round(value, 2)
                # Recalcular promedio solo con los valores disponibles
                temps = [t for t in self.state["sensors"]["temperaturas"][:4] if t is not None]
                if temps:
                    self.state["sensors"]["temperatura"] = round(sum(temps) / len(temps), 2)

            elif slug == "evap_flujo":
                self.state["sensors"]["flujo"] = round(value, 3)

    # ─────────────────────────────────────────
    # Estado y diagnóstico
    # ─────────────────────────────────────────

    def get_status(self) -> dict:
        """
        Returns:
            dict: diagnóstico del listener (conexión, broker, tópicos suscritos
            y segundos desde el último dato).
        """
        return {
            "connected":          self.connected,
            "broker":             self.broker,
            "port":               self.port,
            "topics_subscribed":  list(self._topic_map.keys()),
            "seconds_since_data": self.seconds_since_last_data(),
        }
