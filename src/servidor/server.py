"""
server.py — Servidor central v2
Lab Procesos Industriales — PUCP

CAMBIOS v2:
  - Simulación genera T1..T4 individualmente (no promedio)
  - Batch de DB guarda cada sensor físico por separado
  - WebSocket init history consulta evap_t1 (temperatura real del sensor 1)
  - Endpoint /status corregido (sin inf en JSON)
"""

import asyncio
import json
import logging
import math
import random
import time
import threading
import urllib.request
from contextlib import asynccontextmanager
from datetime import datetime, timezone, timedelta
from typing import List, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Query
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from arm_controller import ArmController
from db import Database
from iot_listener import IoTListener

from config import (
    SERVER_HOST, SERVER_PORT,
    XARM_IP, XARM_SPEED, XARM_ACCEL,
    BASE_IP, BASE_PORT,
    MQTT_BROKER, MQTT_PORT, SENSOR_NODES,
    PG_HOST, PG_PORT, PG_DATABASE, PG_USER, PG_PASSWORD, PG_POOL_MIN, PG_POOL_MAX,
    SIMULATION_MODE, SENSOR_TIMEOUT_SECONDS, DB_SAVE_EVERY,
    BASE_CAM_STREAM_URL, ESP32_CAM_STREAM_URL, ESP32_CAM_CONTROL_URL,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("servidor")

JOINT_LIMITS: List[tuple] = [
    (-360.0, 360.0),
    (-110.0,  10.0),
    (-215.0,   5.0),
    (-360.0, 360.0),
    ( -97.0, 180.0),
]

# ─────────────────────────────────────────────
# Instancias globales
# ─────────────────────────────────────────────

arm = ArmController(ip=XARM_IP, speed=XARM_SPEED, accel=XARM_ACCEL)

db = Database(
    host=PG_HOST, port=PG_PORT, database=PG_DATABASE,
    user=PG_USER, password=PG_PASSWORD,
    min_size=PG_POOL_MIN, max_size=PG_POOL_MAX
)

iot: Optional[IoTListener] = None
MAIN_LOOP = None  # se fija en lifespan; lo usa el callback de reconexión del brazo

# ─────────────────────────────────────────────
# Estado compartido
# ─────────────────────────────────────────────

state = {
    "emergency": False,
    "joints":    [0.0, 0.0, 0.0, 0.0, 0.0],
    "gripper":   "open",
    "base_cmd":  "stop",
    "teleop":    False,
    "sensors": {
        "temperatura":   0.0,           # promedio de T1..T4 (para gráfica en vivo)
        "temperaturas":  [0.0, 0.0, 0.0, 0.0],   # T1, T2, T3, T4 individuales
        "flujo":         0.0,
        "last_update":   None,
    },
    "cameras": {
        "base_stream_url":    BASE_CAM_STREAM_URL,
        "gripper_stream_url": ESP32_CAM_STREAM_URL,
    },
    "ultrasonido": {
        "conectado": False,
        "distancias": {"trasera": None, "derecha": None, "delantera": None, "izquierda": None},
        "stop_cm": {}, "warn_cm": {},
    },
    "conexiones": {
        "xarm": False,
        "base": False,
        "iot":  False,
    }
}

# ─────────────────────────────────────────────
# WebSocket manager
# ─────────────────────────────────────────────

class ConnectionManager:
    def __init__(self):
        """Inicializa el conjunto de clientes WebSocket activos (Set[WebSocket])."""
        self.active: Set[WebSocket] = set()

    async def connect(self, ws: WebSocket):
        """
        Acepta un nuevo cliente WebSocket y lo registra.

        Args:
            ws (WebSocket): conexión entrante ya negociada por FastAPI.
        """
        await ws.accept()
        self.active.add(ws)

    def disconnect(self, ws: WebSocket):
        """Elimina un cliente del registro (no cierra el socket)."""
        self.active.discard(ws)

    async def broadcast(self, data: dict):
        """
        Envía `data` (dict, serializado a JSON) a TODOS los clientes conectados.
        Los sockets muertos se descartan silenciosamente.
        """
        msg = json.dumps(data, default=str)
        dead = set()
        for ws in self.active:
            try:
                await ws.send_text(msg)
            except Exception:
                dead.add(ws)
        self.active -= dead

    async def send(self, ws: WebSocket, data: dict):
        """Envía `data` (dict, serializado a JSON) a UN cliente específico."""
        try:
            await ws.send_text(json.dumps(data, default=str))
        except Exception as e:
            logger.warning(f"Error enviando a cliente: {e}")


manager = ConnectionManager()

# ─────────────────────────────────────────────
# Control del brazo — piezas portadas de la rama del robot
# ─────────────────────────────────────────────

def _arm_connection_changed(connected: bool):
    """Callback del watchdog de ArmController (corre en un hilo daemon)."""
    state["conexiones"]["xarm"] = connected
    if connected:
        logger.info("Brazo reconectado.")
    else:
        logger.warning("Brazo desconectado — el watchdog intentará reconectar.")

    if MAIN_LOOP and MAIN_LOOP.is_running():
        MAIN_LOOP.call_soon_threadsafe(
            lambda: asyncio.create_task(
                manager.broadcast({"type": "arm_connection", "connected": connected})
            )
        )


arm.on_connection_change = _arm_connection_changed


# ─── Verificación de cámaras al arrancar (con reintentos) ───
CAM_CHECK_RETRIES = 5
CAM_CHECK_DELAY = 2.0


def _check_camera_url(url: str, name: str, retries: int = CAM_CHECK_RETRIES) -> bool:
    """
    Verifica que una cámara responda por HTTP.

    Args:
        url (str): URL del stream (ej. http://192.168.1.14:81/stream).
        name (str): etiqueta para los logs ('base' o 'gripper').
        retries (int): número de intentos, con CAM_CHECK_DELAY s entre ellos.
    Returns:
        bool: True si respondió con status < 500, False si agotó reintentos.
    """
    for attempt in range(1, retries + 1):
        try:
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                if resp.status < 500:
                    logger.info(f"Cámara {name} disponible (intento {attempt}).")
                    return True
        except Exception as e:
            logger.warning(f"Cámara {name} — intento {attempt}/{retries} fallido: {e}")
            if attempt < retries:
                time.sleep(CAM_CHECK_DELAY)
    logger.error(f"Cámara {name} no disponible tras {retries} intentos.")
    return False


def _verify_cameras():
    """
    Comprueba en paralelo (hilos) la cámara de la base y la del gripper.

    Returns:
        dict: {'base': bool, 'gripper': bool} disponibilidad de cada cámara.
    """
    results = {}

    def check(url, name):
        results[name] = _check_camera_url(url, name)

    threads = [
        threading.Thread(target=check, args=(BASE_CAM_STREAM_URL, "base"), daemon=True),
        threading.Thread(target=check, args=(ESP32_CAM_STREAM_URL, "gripper"), daemon=True),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return results


# ─── Watchdog del jog XYZ: si no llega xyz_stop a tiempo, frena el brazo ───
XYZ_WATCHDOG_TIMEOUT = 1.0
_xyz_watchdog_task: Optional[asyncio.Task] = None


def _cancel_xyz_watchdog():
    """Cancela el watchdog del jog XYZ si está corriendo."""
    global _xyz_watchdog_task
    if _xyz_watchdog_task and not _xyz_watchdog_task.done():
        _xyz_watchdog_task.cancel()
    _xyz_watchdog_task = None


def _touch_xyz_watchdog():
    """
    Reinicia el watchdog XYZ: si en XYZ_WATCHDOG_TIMEOUT s no llega otro
    comando (o xyz_stop), el brazo se frena solo por seguridad.
    """
    global _xyz_watchdog_task
    _cancel_xyz_watchdog()
    _xyz_watchdog_task = asyncio.create_task(_xyz_watchdog_run())


async def _xyz_watchdog_run():
    """
    Corrutina del watchdog: espera el timeout y detiene el movimiento
    cartesiano del brazo. Se cancela con _cancel_xyz_watchdog().
    """
    try:
        await asyncio.sleep(XYZ_WATCHDOG_TIMEOUT)
        logger.warning("Watchdog XYZ: sin xyz_stop a tiempo, deteniendo brazo.")
        arm.stop_xyz_velocity()
    except asyncio.CancelledError:
        pass


# ─── Estado periódico del brazo: fuente de verdad de conexión, joints y errores ───
async def arm_status_task():
    """
    Tarea periódica (0.5 s): lee el estado real del xArm5 (conexión, joints,
    gripper, errores) y lo difunde a la interfaz como 'arm_status'.
    Es la fuente de verdad del estado del brazo para el dashboard.
    """
    while True:
        await asyncio.sleep(0.5)
        try:
            status = await asyncio.to_thread(arm.get_status)
            state["conexiones"]["xarm"] = bool(status.get("connected"))
            state["joints"] = status.get("joints", state["joints"])
            state["gripper"] = status.get("gripper", state["gripper"])
            await manager.broadcast({"type": "arm_status", **status})
        except Exception as e:
            logger.error(f"Error leyendo estado del brazo: {e}")
            state["conexiones"]["xarm"] = False
            await manager.broadcast({
                "type": "arm_status",
                "connected": False,
                "joints": state["joints"],
                "gripper": state["gripper"],
                "error_code": -1,
                "warn_code": 0,
                "error_msg": f"No se pudo leer estado del brazo: {e}",
                "warn_msg": None,
                "error_action": "reconnect",
                "state": -1,
            })


# ─────────────────────────────────────────────
# Simulación de sensores
# ─────────────────────────────────────────────

_sim_t = 0.0

def _update_simulated_sensors():
    """
    Genera T1..T4 individualmente con variación senoidal + ruido gaussiano.
    Cada sensor tiene una fase ligeramente distinta para simular
    puntos distintos del proceso.
    """
    global _sim_t
    _sim_t += 0.05

    # Base oscilatoria — simula calentamiento/enfriamiento del proceso
    base = 75 + 15 * math.sin(_sim_t * 0.4)

    # Cada sensor tiene su propio offset de fase para diferenciarse
    t1 = round(base + 3.0 + random.gauss(0, 0.8), 1)   # entrada — más caliente
    t2 = round(base - 2.5 + random.gauss(0, 0.6), 1)   # salida — un poco más fría
    t3 = round(base + 1.5 + random.gauss(0, 0.7), 1)   # condensador entrada
    t4 = round(base - 4.0 + random.gauss(0, 0.6), 1)   # condensador salida — más fría

    state["sensors"]["temperaturas"] = [t1, t2, t3, t4]
    state["sensors"]["temperatura"]  = round((t1 + t2 + t3 + t4) / 4.0, 1)
    state["sensors"]["flujo"]        = round(
        max(0, 1.0 + 0.6 * math.sin(_sim_t * 0.7 + 1) + random.gauss(0, 0.05)), 3
    )


def _should_simulate() -> bool:
    """
    Returns:
        bool: True solo si SIMULATION_MODE está activo en config.py.
        (Ya no se simula al perder el IoT, para no mostrar datos falsos.)
    """
    # Solo se simula si el modo simulación está activado explícitamente en config.
    # Antes también se simulaba al desconectarse el IoT, lo que mostraba valores
    # FALSOS de temperatura/flujo con el LED en rojo. Eso ya no ocurre.
    return bool(SIMULATION_MODE)

# ─────────────────────────────────────────────
# Tarea de broadcast periódico
# ─────────────────────────────────────────────

_db_counter = 0

async def sensor_broadcast_task():
    """
    Tarea periódica (0.5 s) de sensores:
      1. Si hay simulación activa, genera datos sintéticos.
      2. Valida frescura de datos reales (iot conectado y < SENSOR_TIMEOUT_SECONDS).
      3. Difunde 'sensors' a la interfaz (null si no hay datos válidos).
      4. Cada DB_SAVE_EVERY ciclos guarda T1..T4 y flujo en PostgreSQL.
    """
    global _db_counter

    while True:
        await asyncio.sleep(0.5)

        simulate = _should_simulate()
        if simulate:
            _update_simulated_sensors()

        # ¿Los datos son reales y frescos? (IoT conectado y con datos recientes)
        iot_fresh = bool(
            state["conexiones"]["iot"]
            and iot
            and iot.seconds_since_last_data() < SENSOR_TIMEOUT_SECONDS
        )
        data_valid = simulate or iot_fresh

        if data_valid:
            sensors_out = state["sensors"]
        else:
            # Sin datos reales ni simulación: NO inventar valores. Se envía
            # null para que la interfaz muestre "N/A / No conectado".
            sensors_out = {
                "temperatura":  None,
                "temperaturas": [None, None, None, None, None],
                "flujo":        None,
            }

        await manager.broadcast({
            "type":       "sensors",
            "data":       sensors_out,
            "conexiones": state["conexiones"],
            "simulated":  simulate,
            "valid":      data_valid,
        })

        _db_counter += 1
        if _db_counter >= DB_SAVE_EVERY:
            _db_counter = 0
            if data_valid and db.connected:
                try:
                    temps = state["sensors"]["temperaturas"]
                    # Guardar cada sensor físico por separado
                    batch = [
                        {"node_slug": "nodo_1", "variable_slug": "evap_t1",   "value": temps[0]},
                        {"node_slug": "nodo_1", "variable_slug": "evap_t2",   "value": temps[1]},
                        {"node_slug": "nodo_1", "variable_slug": "evap_t3",   "value": temps[2]},
                        {"node_slug": "nodo_1", "variable_slug": "evap_t4",   "value": temps[3]},
                        {"node_slug": "nodo_1", "variable_slug": "evap_flujo","value": state["sensors"]["flujo"]},
                    ]
                    await db.insert_reading_batch(batch)
                except Exception as e:
                    logger.error(f"Error guardando en DB: {e}")

# ─────────────────────────────────────────────
# Helpers de hardware
# ─────────────────────────────────────────────

def _send_base_to_raspberry(cmd: str):
    """
    Envía un comando de movimiento a la Raspberry Pi de la base (bloqueante).

    Args:
        cmd (str): 'fwd' | 'bwd' | 'left' | 'right' | 'stop'.
    Efecto: POST JSON {'cmd': cmd} a http://BASE_IP:BASE_PORT/base y
    actualiza state['conexiones']['base'].
    """
    url  = f"http://{BASE_IP}:{BASE_PORT}/base"
    data = json.dumps({"cmd": cmd}).encode("utf-8")
    req  = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=1.0) as response:
            logger.info(f"Raspberry respondió: {response.read().decode('utf-8')}")
            state["conexiones"]["base"] = True
    except Exception as e:
        logger.error(f"No se pudo enviar comando a Raspberry: {e}")
        state["conexiones"]["base"] = False

async def send_base_to_raspberry(cmd: str):
    """Versión async de _send_base_to_raspberry (se ejecuta en un hilo)."""
    await asyncio.to_thread(_send_base_to_raspberry, cmd)


def _send_teleop_to_raspberry(active: bool):
    """
    Arma/desarma la teleoperación en la base (bloqueante).

    Args:
        active (bool): True arma el modo teleoperado, False lo desarma.
    """
    url  = f"http://{BASE_IP}:{BASE_PORT}/teleop"
    data = json.dumps({"active": active}).encode("utf-8")
    req  = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"},
        method="POST"
    )
    try:
        with urllib.request.urlopen(req, timeout=1.5) as response:
            logger.info(f"Teleop -> base: {response.read().decode('utf-8')}")
            state["conexiones"]["base"] = True
    except Exception as e:
        logger.error(f"No se pudo enviar teleop a la base: {e}")
        state["conexiones"]["base"] = False


async def send_teleop_to_raspberry(active: bool):
    """Versión async de _send_teleop_to_raspberry (se ejecuta en un hilo)."""
    await asyncio.to_thread(_send_teleop_to_raspberry, active)


# ─────────────────────────────────────────────
# Ultrasonidos del Nano (leídos por la Raspberry en /ultrasonido)
# ─────────────────────────────────────────────
def _poll_ultrasonido():
    """
    Consulta GET /ultrasonido en la Raspberry de la base y actualiza
    state['ultrasonido'] (distancias en cm por lado, umbrales stop/warn).
    Si falla, marca el módulo como desconectado (no lanza excepción).
    """
    url = f"http://{BASE_IP}:{BASE_PORT}/ultrasonido"
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=1.0) as response:
            data = json.loads(response.read().decode("utf-8"))
        state["ultrasonido"] = {
            "conectado":  data.get("conectado", False),
            "distancias": data.get("distancias", state["ultrasonido"]["distancias"]),
            "stop_cm":    data.get("stop_cm", {}),
            "warn_cm":    data.get("warn_cm", {}),
        }
        state["conexiones"]["base"] = True
    except Exception as e:
        logger.debug(f"No se pudo leer ultrasonido de la base: {e}")
        state["ultrasonido"]["conectado"] = False


def _obstacle_blocks_command(data: dict, cmd: str) -> bool:
    """
    Decide si un obstáculo bloquea el comando de movimiento actual.

    Args:
        data (dict): estado de ultrasonido (state['ultrasonido']).
        cmd (str): comando en curso ('fwd'|'bwd'|'left'|'right').
    Returns:
        bool: True si la distancia en la dirección del movimiento es menor o
        igual al umbral stop_cm de ese lado (por defecto 20 cm).
    """
    direction = {"fwd": "delantera", "bwd": "trasera", "left": "izquierda", "right": "derecha"}.get(cmd)
    if not direction or not data.get("conectado"):
        return False
    value = (data.get("distancias") or {}).get(direction)
    stop = (data.get("stop_cm") or {}).get(direction, 20)
    try:
        return float(value) <= float(stop)
    except (TypeError, ValueError):
        return False

async def ultrasonido_task():
    """
    Tarea periódica (0.3 s): sondea los ultrasonidos y, si hay obstáculo en
    la dirección del movimiento, ejecuta una parada de seguridad de la base
    (stop + desarme de teleop + evento 'base_safety_stop' + registro en DB).
    """
    while True:
        await asyncio.sleep(0.3)
        await asyncio.to_thread(_poll_ultrasonido)
        if state["base_cmd"] != "stop" and _obstacle_blocks_command(state["ultrasonido"], state["base_cmd"]):
            blocked_cmd = state["base_cmd"]
            state["base_cmd"] = "stop"
            state["teleop"] = False
            await send_base_to_raspberry("stop")
            await manager.broadcast({"type": "base_safety_stop", "blocked_cmd": blocked_cmd, "reason": "obstacle"})
            await manager.broadcast({"type": "teleop", "active": False})
            if db.connected:
                await db.insert_event("seguridad_base", f"Obstáculo detectado durante {blocked_cmd}; base detenida")
        await manager.broadcast({
            "type": "ultrasonido",
            "data": state["ultrasonido"],
            "conexiones": state["conexiones"],
        })

def _send_gripper_to_esp32(action: str):
    """
    Envía la acción del gripper al ESP32-CAM (bloqueante).

    Args:
        action (str): 'open' | 'close' — GET a ESP32_CAM_CONTROL_URL?action=...
    """
    url = f"{ESP32_CAM_CONTROL_URL}?action={action}"
    try:
        with urllib.request.urlopen(url, timeout=2) as response:
            logger.info(f"ESP32-CAM respondió: {response.read().decode('utf-8')}")
    except Exception as e:
        logger.error(f"No se pudo enviar gripper al ESP32-CAM: {e}")

async def send_gripper_to_esp32(action: str):
    """Versión async de _send_gripper_to_esp32 (se ejecuta en un hilo)."""
    await asyncio.to_thread(_send_gripper_to_esp32, action)

def validate_joints(values) -> Optional[str]:
    """
    Valida una lista de 5 ángulos de joint contra JOINT_LIMITS.

    Args:
        values (list[float]): ángulos J1..J5 en grados.
    Returns:
        Optional[str]: None si es válida; mensaje de error en caso contrario.
    Ejemplo:
        >>> validate_joints([0, 0, 0, 0, 0])   # -> None (válido)
    """
    if not isinstance(values, list) or len(values) != 5:
        return "Se requieren exactamente 5 valores de joint"
    for i, (val, (lo, hi)) in enumerate(zip(values, JOINT_LIMITS)):
        if not isinstance(val, (int, float)):
            return f"Joint {i+1}: valor no numérico ({val!r})"
        if not (lo <= val <= hi):
            return f"Joint {i+1}: {val}° fuera de rango [{lo}, {hi}]"
    return None

# ─────────────────────────────────────────────
# Lifespan
# ─────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Ciclo de vida del servidor (FastAPI lifespan).
    Arranque: conecta PostgreSQL, xArm5, verifica cámaras, lanza las tareas
    periódicas y el listener MQTT. Apagado: cancela tareas, frena el brazo
    y cierra conexiones de forma ordenada.
    """
    global iot, MAIN_LOOP
    MAIN_LOOP = asyncio.get_running_loop()

    logger.info("Iniciando servidor...")
    logger.info(f"Modo simulación: {'ACTIVO' if SIMULATION_MODE else 'INACTIVO (datos reales)'}")

    await db.connect()
    if db.connected:
        await db.upsert_masters(SENSOR_NODES)
        await db.insert_event("sistema", "Servidor iniciado")
    else:
        logger.warning("PostgreSQL no disponible.")

    arm.connect()
    state["conexiones"]["xarm"] = arm.connected
    real_joints = arm.get_joints()
    if real_joints:
        state["joints"] = real_joints

    logger.info("Verificando cámaras...")
    await asyncio.to_thread(_verify_cameras)

    task = asyncio.create_task(sensor_broadcast_task())
    arm_task = asyncio.create_task(arm_status_task())
    ultra_task = asyncio.create_task(ultrasonido_task())

    iot = IoTListener(
        broker=MQTT_BROKER,
        port=MQTT_PORT,
        state=state,
        nodes_config=SENSOR_NODES,
        on_reading=db.insert_reading if db.connected else None,
    )
    await iot.start()

    logger.info(f"Servidor listo en http://{SERVER_HOST}:{SERVER_PORT}")
    yield

    task.cancel()
    arm_task.cancel()
    ultra_task.cancel()
    _cancel_xyz_watchdog()
    arm.stop_xyz_velocity()
    if iot:
        await iot.stop()
    if db.connected:
        await db.insert_event("sistema", "Servidor detenido")
        await db.disconnect()
    arm.disconnect()
    logger.info("Servidor detenido.")


app = FastAPI(title="Lab IoT Robot Server v2", lifespan=lifespan)
app.mount("/static", StaticFiles(directory="static"), name="static")

# ─────────────────────────────────────────────
# Rutas
# ─────────────────────────────────────────────

@app.get("/")
async def root():
    """GET / — sirve la interfaz web (static/index.html)."""
    return FileResponse("static/index.html")

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """
    WS /ws — canal principal de la interfaz.
    Al conectar envía 'init' (estado completo + últimos 30 min de historial
    de T1) y luego atiende los mensajes del cliente con handle_message().
    Al desconectar, frena el brazo por seguridad.
    """
    await manager.connect(ws)

    history = []
    if db.connected:
        now = datetime.now(timezone.utc)
        # Obtener las últimas 60 lecturas de T1 para poblar la gráfica en vivo
        raw = await db.query_readings(
            machine_slug="evaporador",
            variable_slug="evap_t1",
            since=now - timedelta(minutes=30),
            limit=60,
        )
        for r in reversed(raw):
            ts = r["recorded_at"]
            history.append({
                "temperatura": r["value"],
                "flujo": 0.0,
                "timestamp": ts.isoformat() if hasattr(ts, "isoformat") else str(ts)
            })

    await manager.send(ws, {
        "type":    "init",
        "state":   state,
        "history": history
    })

    try:
        while True:
            raw = await ws.receive_text()
            await handle_message(ws, json.loads(raw))
    except WebSocketDisconnect:
        manager.disconnect(ws)
        _cancel_xyz_watchdog()
        arm.stop_xyz_velocity()
    except Exception as e:
        logger.error(f"Error en WebSocket: {e}")
        manager.disconnect(ws)
        _cancel_xyz_watchdog()
        arm.stop_xyz_velocity()


async def _stop_all():
    """Parada de emergencia global: detiene base, desarma teleop y frena el brazo."""
    state["base_cmd"] = "stop"
    state["teleop"] = False
    arm.emergency_stop()
    try:
        await send_teleop_to_raspberry(False)
    except Exception:
        pass

async def handle_message(ws: WebSocket, msg: dict):
    """
    Despachador de mensajes WebSocket de la interfaz.

    Args:
        ws (WebSocket): cliente que envió el mensaje.
        msg (dict): {'type': str, ...} — tipos soportados:
          emergency, initial_position/home, joint_jog, joints, gripper,
          base, teleop, xyz_start, xyz_stop, j1, j5, arm_clear_error,
          arm_status_request.
    Casi todos verifican state['emergency'] antes de mover hardware.
    """
    msg_type = msg.get("type")

    if msg_type == "emergency":
        state["emergency"] = msg.get("active", True)
        if state["emergency"]:
            if db.connected:
                await db.insert_event("emergencia", "Parada de emergencia activada")
            await _stop_all()
        else:
            if db.connected:
                await db.insert_event("emergencia", "Emergencia desactivada")
            arm.resume_after_emergency()
        await manager.broadcast({"type": "emergency", "active": state["emergency"]})

    elif msg_type in ("initial_position", "arm_home", "home"):
        if state["emergency"]:
            await manager.send(ws, {
                "type": "error",
                "msg": "No se puede mover el brazo: la emergencia está activa."
            })
            return

        logger.info("Comando INITIAL POSITION recibido desde la interfaz")
        ok = await asyncio.to_thread(arm.go_initial_position, True)
        status = await asyncio.to_thread(arm.get_status)
        state["joints"] = status.get("joints", state["joints"])

        await manager.broadcast({
            "type": "initial_position",
            "ok": ok,
            "joints": state["joints"],
            "msg": (
                "Posición inicial ejecutada"
                if ok
                else "El xArm rechazó el movimiento a la posición inicial"
            )
        })

        if db.connected:
            await db.insert_event(
                "brazo",
                "Posición inicial ejecutada"
                if ok
                else "Fallo al ejecutar la posición inicial"
            )

        if not ok:
            await manager.send(ws, {
                "type": "error",
                "msg": (
                    "No se pudo ejecutar la posición inicial. "
                    "Revisa el estado y los errores del xArm."
                )
            })

    elif msg_type == "joint_jog":
        if state["emergency"]:
            await manager.send(ws, {"type": "error", "msg": "Emergencia activa"})
            return

        try:
            index = int(msg.get("index", -1))
            delta = float(msg.get("delta", 0))
        except (TypeError, ValueError):
            await manager.send(ws, {"type": "error", "msg": "Comando de joint inválido"})
            return

        if index not in (0, 3, 4):
            await manager.send(ws, {"type": "error", "msg": "Solo se permite mover J1, J4 y J5"})
            return

        ok = await asyncio.to_thread(arm.jog_joint, index, delta)
        real = await asyncio.to_thread(arm.get_joints)
        if real and len(real) >= 5:
            state["joints"] = real[:5]

        await manager.broadcast({
            "type": "joint_jog_result",
            "index": index,
            "delta": delta,
            "ok": ok,
            "joints": state["joints"],
        })
        await manager.broadcast({"type": "joints", "values": state["joints"], "ok": ok})

        if not ok:
            status = await asyncio.to_thread(arm.get_status)
            await manager.send(ws, {
                "type": "error",
                "msg": status.get("error_msg") or "El xArm rechazó el movimiento del joint",
            })

    elif msg_type == "joints":
        if state["emergency"]:
            await manager.send(ws, {"type": "error", "msg": "Emergencia activa"})
            return
        values = msg.get("values")
        error  = validate_joints(values)
        if error:
            await manager.send(ws, {"type": "error", "msg": f"Joints inválidos: {error}"})
            return
        state["joints"] = values
        arm.set_joints(values)
        await manager.broadcast({"type": "joints", "values": values})

    elif msg_type == "gripper":
        if state["emergency"]:
            return
        action = msg.get("action", "open")
        state["gripper"] = action
        await send_gripper_to_esp32(action)
        if db.connected:
            await db.insert_event("gripper", action)
        await manager.broadcast({"type": "gripper", "action": action})

    elif msg_type == "base":
        if state["emergency"]:
            return
        cmd = msg.get("cmd", "stop")
        state["base_cmd"] = cmd
        await send_base_to_raspberry(cmd)
        await manager.broadcast({"type": "base", "cmd": cmd})

    elif msg_type == "teleop":
        active = bool(msg.get("active", False))
        if state["emergency"]:
            active = False
        state["teleop"] = active
        await send_teleop_to_raspberry(active)
        if db.connected:
            await db.insert_event("base", "Teleoperación armada" if active else "Teleoperación desarmada")
        await manager.broadcast({"type": "teleop", "active": active})

    elif msg_type == "xyz_start":
        if state["emergency"]:
            return
        vx = msg.get("vx", 0)
        vy = msg.get("vy", 0)
        vz = msg.get("vz", 0)
        arm.move_xyz_velocity(vx=vx, vy=vy, vz=vz)
        _touch_xyz_watchdog()

    elif msg_type == "xyz_stop":
        arm.stop_xyz_velocity()
        _cancel_xyz_watchdog()

    elif msg_type == "j1":
        if state["emergency"]:
            return
        arm.set_joint1(msg["value"])

    elif msg_type == "j5":
        if state["emergency"]:
            return
        arm.set_joint5(msg["value"])

    elif msg_type == "arm_clear_error":
        ok = arm.clear_errors()
        error_info = arm.get_error_info()
        await manager.broadcast({
            "type": "arm_error",
            "error_code": error_info["error_code"],
            "warn_code":  error_info["warn_code"],
            "error_msg":  error_info["error_msg"],
            "warn_msg":   error_info["warn_msg"],
            "action":     error_info["action"],
            "cleared":    ok,
        })
        if db.connected:
            detalle = "Error limpiado exitosamente" if ok else f"No se pudo limpiar error {error_info['error_code']}"
            await db.insert_event("brazo", detalle)

    elif msg_type == "arm_status_request":
        status = await asyncio.to_thread(arm.get_status)
        state["conexiones"]["xarm"] = bool(status.get("connected"))
        state["joints"] = status.get("joints", state["joints"])
        state["gripper"] = status.get("gripper", state["gripper"])
        await manager.send(ws, {"type": "arm_status", **status})



@app.get("/arm/clear_error")
async def arm_clear_error():
    """Limpia errores del brazo por HTTP (útil para diagnóstico)."""
    ok = arm.clear_errors()
    error_info = arm.get_error_info()
    if db.connected:
        await db.insert_event("brazo", f"clear_error via HTTP — {'ok' if ok else 'fallo'}")
    return {"ok": ok, **error_info}


@app.get("/status")
async def status():
    """
    GET /status — resumen JSON del sistema: clientes, emergencia, sensores,
    cámaras, conexiones, modo simulación y frescura de datos IoT.
    """
    # Sanitizar float("inf") antes de serializar
    raw_iot_sec = iot.seconds_since_last_data() if iot else None
    iot_seconds = None if (raw_iot_sec is None or raw_iot_sec == float("inf")) \
                  else round(raw_iot_sec, 1)

    return JSONResponse(content={
        "ok":                  True,
        "clientes_conectados": len(manager.active),
        "emergency":           state["emergency"],
        "sensores":            state["sensors"],
        "cameras":             state["cameras"],
        "conexiones":          state["conexiones"],
        "simulation_mode":     SIMULATION_MODE,
        "simulating":          _should_simulate(),
        "db_connected":        db.connected,
        "iot_seconds_since_data": iot_seconds,
    })

@app.get("/api/machines")
async def api_machines():
    """GET /api/machines — lista de máquinas registradas en la DB."""
    machines = await db.query_machines() if db.connected else []
    return JSONResponse(content={"data": machines})

@app.get("/api/nodes")
async def api_nodes(machine: Optional[str] = Query(None)):
    """GET /api/nodes?machine= — nodos sensores (filtro opcional por máquina)."""
    nodes = await db.query_nodes(machine_slug=machine) if db.connected else []
    return JSONResponse(content={"data": nodes})

@app.get("/api/variables")
async def api_variables():
    """
    Retorna las variables registradas, opcionalmente con la máquina a la
    que pertenecen. El frontend de historial las usa para poblar los filtros.
    """
    if not db.connected:
        return JSONResponse(content={"data": []})
    try:
        async with db._pool.acquire() as conn:
            rows = await conn.fetch("""
                SELECT DISTINCT
                    vt.slug, vt.name, vt.unit,
                    m.slug  AS machine_slug,
                    m.name  AS machine_name,
                    n.slug  AS node_slug
                FROM variable_types vt
                JOIN sensor_readings sr ON sr.variable_id = vt.id
                JOIN sensor_nodes    n  ON n.id = sr.node_id
                JOIN machines        m  ON m.id = n.machine_id
                ORDER BY m.slug, vt.slug
            """)
            return JSONResponse(content={"data": [dict(r) for r in rows]})
    except Exception as e:
        logger.error(f"Error en api_variables: {e}")
        return JSONResponse(content={"data": []})

@app.get("/history/readings")
async def history_readings(
    machine:  Optional[str] = Query(None),
    node:     Optional[str] = Query(None),
    variable: Optional[str] = Query(None),
    since:    Optional[str] = Query(None),
    until:    Optional[str] = Query(None),
    limit:    int            = Query(500),
    agg:      Optional[str] = Query(None),
):
    """
    GET /history/readings — historial de lecturas con filtros.

    Query params: machine, node, variable (slugs), since/until (ISO-8601),
    limit (int, default 500), agg ('1min'|'5min'|... agregación opcional).
    Returns: JSON {'total': int, 'data': [{recorded_at, value, ...}]}.
    """
    if not db.connected:
        return JSONResponse(content={"error": "Base de datos no disponible"}, status_code=503)

    rows = await db.query_readings(
        machine_slug=machine, node_slug=node, variable_slug=variable,
        since=_parse_dt(since), until=_parse_dt(until), limit=limit, agg=agg,
    )
    for r in rows:
        if hasattr(r.get("recorded_at"), "isoformat"):
            r["recorded_at"] = r["recorded_at"].isoformat()

    return JSONResponse(content={"total": len(rows), "data": rows})

@app.get("/history/events")
async def history_events(limit: int = Query(50)):
    """
    GET /history/events?limit= — últimos eventos del sistema (emergencias,
    movimientos, paradas de seguridad, etc.).
    """
    if not db.connected:
        return JSONResponse(content={"error": "Base de datos no disponible"}, status_code=503)
    rows = await db.query_events(limit=limit)
    for r in rows:
        if hasattr(r.get("recorded_at"), "isoformat"):
            r["recorded_at"] = r["recorded_at"].isoformat()
    return JSONResponse(content={"total": len(rows), "data": rows})

@app.get("/history/stats")
async def history_stats(
    machine:  Optional[str] = Query(None),
    variable: Optional[str] = Query(None),
    since:    Optional[str] = Query(None),
    until:    Optional[str] = Query(None),
):
    """
    GET /history/stats — estadísticas (min/max/prom) con filtros de
    máquina, variable y rango de fechas.
    """
    if not db.connected:
        return JSONResponse(content={"error": "Base de datos no disponible"}, status_code=503)
    stats = await db.query_stats(
        machine_slug=machine, variable_slug=variable,
        since=_parse_dt(since), until=_parse_dt(until),
    )
    return JSONResponse(content=stats)

@app.get("/history/export/csv")
async def history_export_csv(
    machine:  Optional[str] = Query(None),
    node:     Optional[str] = Query(None),
    variable: Optional[str] = Query(None),
    since:    Optional[str] = Query(None),
    until:    Optional[str] = Query(None),
    limit:    int            = Query(5000),
):
    """
    GET /history/export/csv — descarga el historial filtrado como CSV
    (hasta `limit` filas, default 5000).
    """
    if not db.connected:
        return JSONResponse(content={"error": "Base de datos no disponible"}, status_code=503)
    csv_content = await db.export_readings_csv(
        machine_slug=machine, node_slug=node, variable_slug=variable,
        since=_parse_dt(since), until=_parse_dt(until), limit=limit,
    )
    filename = f"lab_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    return StreamingResponse(
        iter([csv_content]), media_type="text/csv",
        headers={"Content-Disposition": f"attachment; filename={filename}"}
    )

@app.get("/history/analytics")
async def history_analytics(
    machine: str            = Query("evaporador", description="Slug de la máquina"),
    since:   Optional[str]  = Query(None),
    until:   Optional[str]  = Query(None),
):
    """
    Análisis avanzado de los datos históricos de una máquina.
    Calcula: estadísticas con % tiempo sobre umbral, tendencia lineal,
    spread térmico entre sensores y correlación T1-flujo.
    """
    if not db.connected:
        return JSONResponse(content={"error": "Base de datos no disponible"}, status_code=503)
    try:
        result = await db.query_analytics(
            machine_slug=machine,
            since=_parse_dt(since),
            until=_parse_dt(until),
        )
        return JSONResponse(content=result)
    except Exception as e:
        logger.error(f"Error en analytics: {e}")
        return JSONResponse(content={"error": str(e)}, status_code=500)

def _parse_dt(s: Optional[str]) -> Optional[datetime]:
    """
    Parsea una fecha ISO-8601 a datetime UTC.

    Args:
        s (Optional[str]): ej. '2026-07-12T15:30:00'.
    Returns:
        Optional[datetime]: None si s es vacío o inválido.
    """
    if not s:
        return None
    try:
        dt = datetime.fromisoformat(s)
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        return dt
    except Exception:
        return None

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("server:app", host=SERVER_HOST, port=SERVER_PORT, reload=True)
