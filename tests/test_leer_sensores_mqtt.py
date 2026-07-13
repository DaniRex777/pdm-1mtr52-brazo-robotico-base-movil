"""
test_leer_sensores_mqtt.py — Prueba básica de sensor (rúbrica: "leer un sensor")
Proyecto 1MTR52 — Lab. Procesos Industriales, PUCP

Se suscribe a los tópicos MQTT del nodo sensor (ESP32-S3) e imprime las
lecturas de temperatura (DS18B20 x4) y flujo (YF-B1) durante 30 segundos.
Sirve para verificar la cadena nodo sensor -> broker -> PC sin levantar
el servidor completo.

Requisitos:
    pip install "paho-mqtt<2"
    Broker Mosquitto corriendo y nodo sensor encendido.

Uso:
    python tests/test_leer_sensores_mqtt.py

Salida esperada (cada ~2 s):
    [lab/evaporador/temperatura] {"value":25.31,"t1":25.50,...}
    [lab/evaporador/flujo] {"value":0.000}
"""

import time

import paho.mqtt.client as mqtt

BROKER = "192.168.1.35"   # IP del PC donde corre Mosquitto (ver config.py)
PORT = 1883
TOPICS = [
    "lab/evaporador/temperatura",   # JSON: {"value":prom,"t1":..,"t2":..,"t3":..,"t4":..} [°C]
    "lab/evaporador/flujo",         # JSON: {"value":caudal} [L/min]
    "lab/evaporador/status",        # JSON: {"status":"online","node":"nodo_1"}
]
DURATION_S = 30


def on_connect(client, userdata, flags, rc):
    """Callback de conexión: rc == 0 indica éxito; se suscribe a los tópicos."""
    if rc == 0:
        print(f"Conectado al broker {BROKER}:{PORT}")
        for t in TOPICS:
            client.subscribe(t)
            print(f"  suscrito a {t}")
    else:
        print(f"Fallo de conexión (rc={rc})")


def on_message(client, userdata, msg):
    """Callback por mensaje: imprime tópico y payload crudo (str JSON)."""
    print(f"[{msg.topic}] {msg.payload.decode('utf-8')}")


client = mqtt.Client(client_id="test_lectura_sensores")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, PORT, keepalive=30)

print(f"Escuchando {DURATION_S} s... (Ctrl+C para salir)")
client.loop_start()
try:
    time.sleep(DURATION_S)
except KeyboardInterrupt:
    pass
client.loop_stop()
client.disconnect()
print("Prueba finalizada.")
