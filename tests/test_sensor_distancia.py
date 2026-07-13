"""
test_sensor_distancia.py — Prueba básica del sensor TOF VL53L0X por HTTP
Proyecto 1MTR52 — Lab. Procesos Industriales, PUCP

Consulta 10 veces el endpoint /distance del ESP32-WROVER dedicado al
sensor de distancia y muestra los valores filtrados/calibrados en mm.

Requisitos: solo librería estándar (urllib). El ESP32 del TOF debe estar
encendido y en la misma red.

Uso:
    python tests/test_sensor_distancia.py
"""

import time
import urllib.request

TOF_URL = "http://192.168.1.17/distance"  # IP estática del sketch sensor_distancia_tof
N_LECTURAS = 10

for i in range(1, N_LECTURAS + 1):
    try:
        with urllib.request.urlopen(TOF_URL, timeout=2.0) as resp:
            valor = resp.read().decode("utf-8").strip()  # mm (str); "-1" = error
        print(f"Lectura {i:2d}: {valor} mm")
    except Exception as e:
        print(f"Lectura {i:2d}: sin respuesta ({e})")
    time.sleep(0.5)
