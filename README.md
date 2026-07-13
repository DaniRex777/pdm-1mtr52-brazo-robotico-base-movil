# Sistema de teleoperación robótica e IoT — Evaporador de vacío

Proyecto **1MTR52 — Proyecto de Diseño Mecatrónico** (PUCP, 2026-1).
Plataforma móvil con brazo manipulador teleoperado e interfaz IoT para el
monitoreo y control remoto de un evaporador de vacío del Laboratorio de
Procesos Industriales de la PUCP.

## Estructura del proyecto

```
sistema-teleoperacion-evaporador-iot/
├── README.md
├── requirements.txt            # dependencias Python con versiones
├── environment.yml             # entorno virtual (conda)
├── docs/
│   └── logica_funcionamiento.md    # lógica general + pseudocódigo
├── src/
│   ├── servidor/               # SOFTWARE CENTRAL (PC del laboratorio)
│   │   ├── main.py             # ★ ARCHIVO PRINCIPAL — punto de entrada
│   │   ├── server.py           # aplicación FastAPI (WebSocket, rutas, tareas)
│   │   ├── arm_controller.py   # módulo de control del brazo xArm5
│   │   ├── iot_listener.py     # módulo suscriptor MQTT (nodo sensor)
│   │   ├── db.py               # módulo de persistencia (PostgreSQL)
│   │   ├── config.py           # constantes: IPs, puertos, tópicos, umbrales
│   │   └── static/index.html   # interfaz web (dashboard)
│   └── firmware/               # FIRMWARE de los microcontroladores (Arduino IDE)
│       ├── nodo_sensor_mqtt/       # ESP32-S3: 4x DS18B20 + flujo YF-B1 → MQTT
│       ├── sensor_distancia_tof/   # ESP32-WROVER: VL53L0X 20 Hz filtrado → HTTP
│       ├── sensores_ultrasonido/   # Arduino: 4x HC-SR04 → Serial (anticolisión)
│       └── camara_gripper/         # ESP32-CAM: stream de video + servo del gripper
└── tests/
    ├── test_mover_brazo.py         # prueba básica: mover un motor (xArm5)
    ├── test_leer_sensores_mqtt.py  # prueba básica: leer sensores por MQTT
    └── test_sensor_distancia.py    # prueba básica: leer el TOF por HTTP
```

## Hardware empleado

| Equipo | Función | Código asociado |
|---|---|---|
| PC (Windows) | Servidor central, broker MQTT, PostgreSQL, interfaz web | `src/servidor/` |
| Brazo UFactory xArm5 | Manipulador de 5 GDL (IP 192.168.1.228) | `arm_controller.py` |
| Raspberry Pi (base móvil) | Recibe comandos de la base y expone ultrasonidos y cámara | endpoints `/base`, `/teleop`, `/ultrasonido` |
| ESP32-S3 | Nodo sensor: 4x DS18B20 (OneWire, GPIO4) + flujo YF-B1 (GPIO5) | `nodo_sensor_mqtt.ino` |
| ESP32-WROVER-IE | Sensor de distancia TOF VL53L0X (I2C: SDA 21, SCL 22) | `sensor_distancia_tof.ino` |
| ESP32-CAM (AI-Thinker) | Cámara del gripper + servo (GPIO13) | `camara_gripper.ino` |
| Arduino (Nano) | 4x HC-SR04 (TRIG 2,4,6,8 / ECHO 3,5,7,9) anticolisión | `sensores_ultrasonido.ino` |
| Router TP-Link | Red local del laboratorio (192.168.1.0/24) | — |

## Requisitos e instalación

### 1. Servidor central (Python 3.11+)

```bash
# Opción A — venv + pip
python -m venv .venv
.venv\Scripts\activate            # Windows  (Linux/Mac: source .venv/bin/activate)
pip install -r requirements.txt

# Opción B — conda
conda env create -f environment.yml
conda activate lab-iot-teleop
```

Servicios externos en el PC:
- **Mosquitto** (broker MQTT, puerto 1883): https://mosquitto.org/download/
- **PostgreSQL 16** (puerto 5432): crear la base `labiot` con usuario/clave de
  `config.py`. Las tablas se crean solas al primer arranque (`db.py`).
  Si PostgreSQL o Mosquitto no están disponibles, el servidor arranca igual
  y desactiva ese subsistema (modo degradado).

### 2. Firmware (Arduino IDE 2.x)

1. Instalar el soporte de placas **esp32 by Espressif Systems** (Boards Manager).
2. Instalar librerías (Library Manager), versiones usadas:
   - `PubSubClient` 2.8 (Nick O'Leary) — nodo sensor
   - `OneWire` 2.3.8 y `DallasTemperature` 4.0.4 — nodo sensor
   - `VL53L0X` 1.3.1 (Pololu) — sensor TOF
   - `ESP32Servo` 3.0.6 — cámara/gripper
3. **Drivers USB-serial**: CP210x (Silicon Labs) para ESP32 DevKit/WROVER y
   CH340 para clones/Arduino Nano. Sin ellos el puerto COM no aparece.
4. Antes de subir cada sketch, editar la sección `CONFIGURACIÓN` del `.ino`
   (SSID/clave WiFi, IPs). Placas: ESP32S3 Dev Module (nodo sensor),
   ESP32 Wrover Module (TOF), AI Thinker ESP32-CAM (cámara).

## Cómo ejecutar

```bash
# 1. Encender router, brazo, base móvil y los ESP32 (se conectan solos)
# 2. En el PC: verificar Mosquitto y PostgreSQL activos
# 3. Lanzar el servidor
cd src/servidor
python main.py
# 4. Abrir la interfaz: http://localhost:8000
```

## Pruebas básicas

```bash
python tests/test_leer_sensores_mqtt.py   # leer sensores (temperatura/flujo)
python tests/test_sensor_distancia.py     # leer el sensor TOF
python tests/test_mover_brazo.py          # mover un motor del brazo (J1 +10°)
```

## Lógica general de funcionamiento

Ver [`docs/logica_funcionamiento.md`](docs/logica_funcionamiento.md)
(pseudocódigo de cada subsistema). Resumen:

1. El **nodo sensor** publica temperatura y flujo por MQTT cada 2 s.
2. El **servidor** (`main.py` → `server.py`) escucha MQTT, guarda en
   PostgreSQL y retransmite todo a la interfaz por WebSocket cada 0.5 s.
3. La **interfaz web** muestra cámaras, sensores e historial, y envía los
   comandos de teleoperación (brazo, base, gripper).
4. El servidor traduce los comandos: xArm5 por SDK, base por HTTP a la
   Raspberry, gripper por HTTP al ESP32-CAM.
5. Los **ultrasonidos** frenan la base automáticamente ante obstáculos
   (parada de seguridad), y la **parada de emergencia** detiene todo.

## Autores

Grupo 09M4 — 1MTR52 PUCP 2026-1: A. Negron, F. Romero, H. D. Huatuco,
S. Galvez, M. Cuenca, R. Torres, D. Solari.
