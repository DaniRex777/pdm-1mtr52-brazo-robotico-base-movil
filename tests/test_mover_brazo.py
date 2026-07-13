"""
test_mover_brazo.py — Prueba básica de actuador (rúbrica: "mover un motor")
Proyecto 1MTR52 — Lab. Procesos Industriales, PUCP

Verifica la comunicación con el brazo xArm5 y ejecuta un movimiento
simple y seguro de J1 (+10°) a baja velocidad.

Requisitos:
    pip install xArm-Python-SDK
    El brazo debe estar encendido y accesible en la red (editar XARM_IP).

Uso:
    python tests/test_mover_brazo.py

Salida esperada:
    connected: True, state/error/warn en 0, y el brazo mueve J1 a 10°.
"""

from xarm.wrapper import XArmAPI

# IP del controlador xArm5 (la misma de src/servidor/config.py)
XARM_IP = "192.168.1.228"

arm = XArmAPI(XARM_IP)

# ── Diagnóstico de conexión ─────────────────────────────────
print("connected:", arm.connected)   # bool — True si el SDK enlazó con el controlador
print("state:",     arm.state)       # int  — 0/2 = listo; 4 = detenido por error
print("error:",     arm.error_code)  # int  — 0 = sin error
print("warn:",      arm.warn_code)   # int  — 0 = sin advertencia

# ── Preparación: limpiar fallas y habilitar movimiento ──────
arm.clean_warn()
arm.clean_error()
arm.motion_enable(True)   # habilita los servos
arm.set_mode(0)           # modo 0 = control de posición
arm.set_state(0)          # estado 0 = listo para moverse

# ── Movimiento de prueba ────────────────────────────────────
# J1 a +10°, resto en 0°. speed en °/s, mvacc en °/s².
# wait=True bloquea hasta terminar. Devuelve 0 si fue aceptado.
ret = arm.set_servo_angle(angle=[10, 0, 0, 0, 0], speed=20, mvacc=100, wait=True)
print("set_servo_angle ->", ret, "(0 = OK)")

arm.disconnect()
