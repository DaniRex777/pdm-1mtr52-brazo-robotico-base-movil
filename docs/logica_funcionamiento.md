# Lógica general de funcionamiento (pseudocódigo)

Complemento del README. Los diagramas de flujo detallados están en el
informe del proyecto (sección 3.5). Aquí se resume la lógica de cada
programa en pseudocódigo.

## 1. Servidor central (`src/servidor/main.py` → `server.py`)

```
INICIO (lifespan)
  conectar PostgreSQL (si falla → modo degradado sin historial)
  conectar brazo xArm5 (watchdog de reconexión en segundo plano)
  verificar cámaras (base y gripper) en paralelo
  lanzar tareas periódicas:
      arm_status_task        (cada 0.5 s)
      sensor_broadcast_task  (cada 0.5 s)
      ultrasonido_task       (cada 0.3 s)
  iniciar IoTListener (suscripción MQTT a lab/evaporador/#)

BUCLE sensor_broadcast_task (cada 0.5 s)
  SI modo simulación → generar T1..T4 y flujo sintéticos
  SI hay datos IoT frescos (< 30 s) O simulación → datos válidos
  SINO → enviar null (la interfaz muestra "No conectado")
  difundir {sensores, conexiones} a todos los clientes WebSocket
  cada 10 ciclos (5 s) → guardar T1..T4 y flujo en PostgreSQL

BUCLE ultrasonido_task (cada 0.3 s)
  leer distancias de la Raspberry (GET /ultrasonido)
  SI la base se mueve hacia un lado con distancia <= stop_cm
      → detener base, desarmar teleop, notificar y registrar evento

AL RECIBIR mensaje WebSocket (handle_message)
  emergency        → detener todo / reanudar
  joints/joint_jog → validar límites articulares y mover xArm5
  xyz_start/stop   → velocidad cartesiana + watchdog de 1 s
  gripper          → GET al ESP32-CAM (/gripper?action=open|close)
  base/teleop      → POST a la Raspberry de la base
```

## 2. Nodo sensor (`src/firmware/nodo_sensor_mqtt/`)

```
SETUP
  iniciar DS18B20 (x4, resolución 10 bits) e interrupción del YF-B1
  conectar WiFi (IP estática) → probar TCP al broker → conectar MQTT

LOOP
  SI WiFi o MQTT caídos → reconectar (reinicio tras timeout)
  cada 1 s   → convertir pulsos del YF-B1 a L/min (11 pulsos/s = 1 L/min)
  cada 2 s   → leer T1..T4, promediar válidas y publicar:
               lab/evaporador/temperatura  {"value","t1".."t4"}
               lab/evaporador/flujo        {"value"}
```

## 3. Sensor de distancia TOF (`src/firmware/sensor_distancia_tof/`)

```
LOOP (cada 50 ms ≈ 20 Hz)
  leer VL53L0X → calibrar (d' = A·d + B) → filtro mediana (5)
  → media móvil (5) → publicar en HTTP /distance (crudo en /raw)
```

## 4. Ultrasonidos anticolisión (`src/firmware/sensores_ultrasonido/`)

```
LOOP
  para cada HC-SR04 (trasera, derecha, delantera, izquierda):
      medir eco (timeout 30 ms; fuera de rango → -1.0)
  enviar por Serial: <trasera,derecha,delantera,izquierda> en cm
  (la Raspberry la lee y la expone en /ultrasonido para el servidor)
```

## 5. Cámara + gripper (`src/firmware/camara_gripper/`)

```
Basado en CameraWebServer (Espressif) +:
  GET /gripper?action=open  → servo GPIO13 a 0°   (abrir)
  GET /gripper?action=close → servo GPIO13 a 180° (cerrar)
Stream MJPEG en :81/stream (usado por el dashboard)
```
