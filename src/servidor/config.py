"""
config.py — Configuración central del sistema
Lab Procesos Industriales — PUCP

CAMBIOS v2:
  - mqtt_prefix corregido: "lab/evaporador" (sin /nodo_1/)
  - variables ahora modelan cada sensor físico por separado (t1..t4 + flujo)
  - mqtt_key indica la clave exacta dentro del JSON publicado por el ESP32
  - MQTT_TOPIC_TEMPERATURA y MQTT_TOPIC_FLUJO actualizados
"""

# ─────────────────────────────────────────────
# Modo de operación
# ─────────────────────────────────────────────
SIMULATION_MODE = False          # True = modo prueba sin hardware
SENSOR_TIMEOUT_SECONDS = 30     # segundos sin datos antes de simular

# ─────────────────────────────────────────────
# Red
# ─────────────────────────────────────────────
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8000

# ─────────────────────────────────────────────
# xArm5
# ─────────────────────────────────────────────
XARM_IP    = "192.168.1.228"
XARM_SPEED = 50
XARM_ACCEL = 200

# ─────────────────────────────────────────────
# Base diferencial
# ─────────────────────────────────────────────
BASE_IP   = "192.168.1.11"
BASE_PORT = 5005

# ─────────────────────────────────────────────
# MQTT — broker
# ─────────────────────────────────────────────
MQTT_BROKER = "127.0.0.1"
MQTT_PORT   = 1883

# ─────────────────────────────────────────────
# Nodos sensores
#
# mqtt_prefix: prefijo exacto del tópico que publica el ESP32.
#   El ESP32 publica en:
#     lab/evaporador/temperatura  → JSON {"value":avg,"t1":x,"t2":x,"t3":x,"t4":x}
#     lab/evaporador/flujo        → JSON {"value":x}
#   Por tanto mqtt_prefix = "lab/evaporador"
#   y el tópico completo = mqtt_prefix + "/" + variable_mqtt_topic
#
# Cada entrada en "variables" representa un sensor físico distinto.
#   slug      → identificador único en la DB (debe ser único globalmente)
#   name      → etiqueta legible que aparece en la interfaz
#   unit      → unidad de medida
#   warn/max  → umbrales de alerta y máximo del sensor
#   mqtt_topic → sufijo del tópico MQTT donde viene este grupo de variables
#   mqtt_key   → clave exacta dentro del JSON de ese tópico
#                ("value" para flujo, "t1".."t4" para temperaturas)
# ─────────────────────────────────────────────
SENSOR_NODES = [
    {
        "node_slug":    "nodo_1",
        "machine_slug": "evaporador",
        "machine_name": "Evaporador al vacío",
        "mqtt_prefix":  "lab/evaporador",
        "variables": [
            {
                "slug":        "evap_t1",
                "name":        "T1 — Evaporador",
                "unit":        "°C",
                "warn":        150.0,
                "max":         200.0,
                "mqtt_topic":  "temperatura",   # tópico: lab/evaporador/temperatura
                "mqtt_key":    "t1",            # clave en el JSON
            },
            {
                "slug":        "evap_t2",
                "name":        "T2 — Evaporador",
                "unit":        "°C",
                "warn":        150.0,
                "max":         200.0,
                "mqtt_topic":  "temperatura",
                "mqtt_key":    "t2",
            },
            {
                "slug":        "evap_t3",
                "name":        "T3 — Evaporador",
                "unit":        "°C",
                "warn":        150.0,
                "max":         200.0,
                "mqtt_topic":  "temperatura",
                "mqtt_key":    "t3",
            },
            {
                "slug":        "evap_t4",
                "name":        "T4 — Evaporador",
                "unit":        "°C",
                "warn":        150.0,
                "max":         200.0,
                "mqtt_topic":  "temperatura",
                "mqtt_key":    "t4",
            },
            {
                "slug":        "evap_flujo",
                "name":        "Flujo másico",
                "unit":        "L/min",
                "warn":        1.8,
                "max":         2.0,
                "mqtt_topic":  "flujo",         # tópico: lab/evaporador/flujo
                "mqtt_key":    "value",
            },
        ]
    },
    # Para agregar la marmita en el futuro, descomentar y ajustar:
    # {
    #     "node_slug":    "nodo_2",
    #     "machine_slug": "marmita",
    #     "machine_name": "Marmita",
    #     "mqtt_prefix":  "lab/marmita",
    #     "variables": [
    #         {"slug":"marm_t1","name":"T1 — Marmita","unit":"°C",
    #          "warn":90.0,"max":120.0,"mqtt_topic":"temperatura","mqtt_key":"t1"},
    #         {"slug":"marm_flujo","name":"Flujo marmita","unit":"L/min",
    #          "warn":1.5,"max":2.0,"mqtt_topic":"flujo","mqtt_key":"value"},
    #     ]
    # },
]

# Tópicos legacy — usados como referencia y en logs
MQTT_TOPIC_TEMPERATURA = "lab/evaporador/temperatura"
MQTT_TOPIC_FLUJO       = "lab/evaporador/flujo"

# ─────────────────────────────────────────────
# PostgreSQL
# ─────────────────────────────────────────────
PG_HOST     = "127.0.0.1"
PG_PORT     = 5432
PG_DATABASE = "labiot"
PG_USER     = "labuser"
PG_PASSWORD = "labpass2026"
PG_POOL_MIN = 2
PG_POOL_MAX = 10

# ─────────────────────────────────────────────
# Almacenamiento histórico
# ─────────────────────────────────────────────
DB_SAVE_EVERY = 10      # cada cuántos ciclos de 500ms guardar (10 = cada 5s)

# ─────────────────────────────────────────────
# Cámaras
# ─────────────────────────────────────────────
ESP32_CAM_IP          = "192.168.1.14"
ESP32_CAM_CONTROL_URL = f"http://{ESP32_CAM_IP}/gripper"
ESP32_CAM_STREAM_URL  = f"http://{ESP32_CAM_IP}:81/stream"
BASE_CAM_STREAM_URL   = "http://192.168.1.11:5000/video"
