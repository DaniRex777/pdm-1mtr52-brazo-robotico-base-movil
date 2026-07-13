"""
main.py — ARCHIVO PRINCIPAL del sistema
Proyecto 1MTR52 — Plataforma móvil con brazo manipulador teleoperado e
interfaz IoT para el monitoreo de un evaporador de vacío.
Lab. Procesos Industriales — PUCP

Punto de entrada del servidor central. Levanta la aplicación FastAPI
definida en server.py, que integra los módulos auxiliares:

    server.py         -> aplicación FastAPI: WebSocket, rutas HTTP y tareas periódicas
    arm_controller.py -> control del brazo xArm5 (xArm-Python-SDK)
    iot_listener.py   -> suscriptor MQTT del nodo sensor (paho-mqtt)
    db.py             -> persistencia en PostgreSQL (asyncpg)
    config.py         -> constantes: IPs, puertos, tópicos MQTT y umbrales

Ejecución (con el entorno virtual activo, desde src/servidor/):

    python main.py

Luego abrir la interfaz web en http://localhost:8000
"""

import uvicorn

from config import SERVER_HOST, SERVER_PORT

if __name__ == "__main__":
    # reload=False en operación normal; usar --reload solo en desarrollo
    uvicorn.run("server:app", host=SERVER_HOST, port=SERVER_PORT)
