"""
run_robot.py — ARCHIVO PRINCIPAL de la base móvil (Raspberry Pi)
Proyecto 1MTR52 — Lab. Procesos Industriales, PUCP

Lanzador de los dos servicios de la base y supervisor de ambos:
    movimiento_base.py -> servidor Flask de movimiento (puerto 5005)
    stream.py          -> servidor Flask de cámaras (puerto 5000)

Si cualquiera de los dos se cae, apaga todo (evita una base "medio viva").

Uso (en la Raspberry Pi, desde src/base/):
    python3 run_robot.py
Detener: Ctrl+C (frena motores y libera GPIO vía los finally de cada módulo).
"""

import subprocess
import signal
import sys
import time

processes = []

def stop_all(sig=None, frame=None):
    """
    Detiene ambos servicios de forma ordenada (SIGTERM y, si no
    responden en 1 s, SIGKILL). Registrado como handler de Ctrl+C.

    Args:
        sig, frame: parámetros estándar de signal (no se usan).
    """
    print("\nDeteniendo servicios...")

    for p in processes:
        if p.poll() is None:
            p.terminate()

    time.sleep(1)

    for p in processes:
        if p.poll() is None:
            p.kill()

    print("Servicios detenidos")
    sys.exit(0)

signal.signal(signal.SIGINT, stop_all)
signal.signal(signal.SIGTERM, stop_all)

print("Iniciando servidor de movimiento...")
p_mov = subprocess.Popen(["python3", "movimiento_base.py"])
processes.append(p_mov)

print("Iniciando servidor de cámara...")
p_cam = subprocess.Popen(["python3", "stream.py"])
processes.append(p_cam)

print("Robot activo:")
print("- Movimiento: http://0.0.0.0:5005")
print("- Cámara:     http://0.0.0.0:5000/video")
print("Presiona CTRL+C para detener todo.")

while True:
    for p in processes:
        if p.poll() is not None:
            print("Uno de los servicios se detuvo. Apagando todo.")
            stop_all()
    time.sleep(1)
