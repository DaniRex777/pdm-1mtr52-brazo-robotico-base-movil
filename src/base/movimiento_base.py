from __future__ import annotations

"""
movimiento_base.py — Servidor de movimiento de la base móvil (Raspberry Pi)
Proyecto 1MTR52 — Lab. Procesos Industriales, PUCP

Recibe por HTTP los comandos de la interfaz (vía server.py del PC) y los
convierte en PWM para los dos motores de la base diferencial:

    POST /base        {"cmd": "fwd"|"bwd"|"left"|"right"|"stop"}
    POST /teleop      {"active": true|false}   (armado de teleoperación)
    GET  /ultrasonido                          (distancias del Nano)
    GET  /status

Seguridad implementada:
    - Puerta de armado: sin teleop armada solo se acepta 'stop'.
    - Ultrasonidos: bloquea avance/retroceso si hay obstáculo bajo el umbral.
    - Baliza (luz + zumbador): fija al armar, parpadea + suena al moverse.

Hardware (pines BCM de la Raspberry Pi):
    Motor izq (BTS7960): RPWM=12, LPWM=13, R_EN=23, L_EN=24
    Motor der (BTS7960): RPWM=18, LPWM=19, R_EN=25, L_EN=26
    Baliza: GPIO16 | Arduino Nano (ultrasonidos): /dev/ttyUSB0 @ 115200

Ejecutar (normalmente lo lanza run_robot.py):
    python3 movimiento_base.py
"""


import math
import time
import threading
import RPi.GPIO as GPIO
from typing import NamedTuple, Optional
from flask import Flask, request, jsonify

# pyserial es opcional: si el Nano no esta conectado o la libreria no
# esta instalada, el servidor de movimiento sigue funcionando igual.
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[ULTRASONIDO] pyserial no encontrado. Instalar con: pip install pyserial")


# ---------- CINEMATICA ----------
B_MM = 346.0
R_MM = 101.0

RPM_MAX = 42.0
PWM_MAX = 100.0
W_MAX_RAD_S = RPM_MAX * (2.0 * math.pi / 60.0)
FACTOR_PWM = PWM_MAX / W_MAX_RAD_S

V_CMD_MM_S = 200.0
W_CMD_RAD_S = 0.35


# ---------- ULTRASONIDOS (ARDUINO NANO) ----------
# Puerto USB del Nano. En la Raspberry suele ser /dev/ttyUSB0 (chip CH340)
# o /dev/ttyACM0. Verificar con:  ls /dev/tty*   o   dmesg | grep tty
NANO_PORT = "/dev/ttyUSB0"
NANO_BAUD = 115200  # debe coincidir con el Serial.begin() del Nano

# Orden de la trama que envia el Nano: <trasera,derecha,delantera,izquierda>
NANO_ORDEN = ["trasera", "derecha", "delantera", "izquierda"]

# Umbrales en cm (ajustar segun el tamano del robot y las pruebas)
# Umbrales en cm POR CARA del robot (ajustar cada uno con pruebas).
# Puedes poner distancias distintas por lado: p. ej. más margen al frente
# porque es donde el robot avanza a mayor velocidad.
OBSTACULO_STOP_CM = {
    "delantera": 50.0,   # bloquea el avance
    "trasera":   50.0,   # bloquea el retroceso
    "derecha":   35.0,
    "izquierda": 35.0,
}
OBSTACULO_WARN_CM = {
    "delantera": 700.0,   # la interfaz muestra alerta (ámbar)
    "trasera":   70.0,
    "derecha":   60.0,
    "izquierda": 60.0,
}


class WheelSpeeds(NamedTuple):
    """Velocidades angulares de las ruedas (rad/s): phi_i izquierda, phi_d derecha."""
    phi_i_rad_s: float
    phi_d_rad_s: float


def inverse_kinematics(v_mm_s: float, w_rad_s: float) -> WheelSpeeds:
    """
    Cinemática inversa de la base diferencial.

    Args:
        v_mm_s (float): velocidad lineal deseada del robot en mm/s.
        w_rad_s (float): velocidad angular deseada en rad/s (+ = horario).
    Returns:
        WheelSpeeds: velocidades de rueda (rad/s) según
        phi = (v ± B·w) / R, con B = 346 mm (entre-eje) y R = 101 mm (radio).
    """
    phi_d = (v_mm_s + B_MM * w_rad_s) / R_MM
    phi_i = (v_mm_s - B_MM * w_rad_s) / R_MM
    return WheelSpeeds(phi_i, phi_d)


def clamp_pwm(pwm: float, limit: float = PWM_MAX) -> float:
    """
    Satura un duty cycle al rango [-limit, +limit] (default ±100 %).

    Args:
        pwm (float): duty solicitado.
    Returns:
        float: duty saturado.
    """
    return max(-limit, min(limit, pwm))


def twist_to_pwm(v_mm_s: float, w_rad_s: float) -> tuple[float, float]:
    """
    Convierte (v, w) del robot a duty PWM de cada motor.

    Args:
        v_mm_s (float), w_rad_s (float): consigna de movimiento.
    Returns:
        tuple[float, float]: (pwm_izq, pwm_der) en % [-100, 100],
        escalados con FACTOR_PWM (100 % = 42 RPM del motorreductor).
    Ejemplo:
        >>> twist_to_pwm(200.0, 0.0)   # avance recto
    """
    speeds = inverse_kinematics(v_mm_s, w_rad_s)

    pwm_i = speeds.phi_i_rad_s * FACTOR_PWM
    pwm_d = speeds.phi_d_rad_s * FACTOR_PWM

    return clamp_pwm(pwm_i), clamp_pwm(pwm_d)


# ---------- LECTOR DE ULTRASONIDOS ----------
class UltrasonicReader:
    """
    Lee en un hilo aparte las tramas <a,b,c,d> que envia el Arduino Nano
    por USB serial y las guarda como distancias {trasera, derecha,
    delantera, izquierda}. Es tolerante a fallos: si el Nano se desconecta
    o la libreria no esta, no rompe el servidor de movimiento.
    """

    def __init__(self, port: str, baud: int, orden: list[str]):
        """
        Args:
            port (str): puerto serial del Nano (ej. '/dev/ttyUSB0').
            baud (int): baudrate (115200, igual que el sketch).
            orden (list[str]): nombres de los 4 sensores en el orden de la trama.
        """
        self.port = port
        self.baud = baud
        self.orden = orden

        self._lock = threading.Lock()
        self._distancias = {nombre: None for nombre in orden}
        self._ts_ultima = 0.0        # marca de tiempo de la ultima lectura valida
        self.conectado = False

        self._ser = None
        self._running = False

    def start(self):
        """Arranca el hilo lector (daemon). Sin pyserial queda desactivado."""
        if not SERIAL_AVAILABLE:
            print("[ULTRASONIDO] pyserial no disponible, lector desactivado")
            return
        self._running = True
        hilo = threading.Thread(target=self._loop, daemon=True)
        hilo.start()

    def stop(self):
        """Detiene el hilo lector y cierra el puerto serial del Nano."""
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass

    def _loop(self):
        """
        Hilo lector: (re)abre el puerto si se cae y extrae tramas <a,b,c,d>
        del buffer, tolerando tramas incompletas o basura.
        """
        buffer = ""
        while self._running:
            # (Re)conexion del puerto serial
            if self._ser is None:
                try:
                    self._ser = serial.Serial(self.port, self.baud, timeout=1.0)
                    self.conectado = True
                    buffer = ""
                    print(f"[ULTRASONIDO] Nano conectado en {self.port} @ {self.baud}")
                except Exception as e:
                    self.conectado = False
                    print(f"[ULTRASONIDO] No se pudo abrir {self.port}: {e}. Reintentando...")
                    time.sleep(2.0)
                    continue

            # Lectura y parseo de tramas
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
                if not data:
                    continue

                buffer += data.decode("utf-8", errors="ignore")

                # Evita que el buffer crezca sin limite si llega basura
                if len(buffer) > 200:
                    buffer = buffer[-200:]

                while True:
                    ini = buffer.find("<")
                    if ini == -1:
                        break
                    fin = buffer.find(">", ini)
                    if fin == -1:
                        buffer = buffer[ini:]  # trama incompleta, esperar mas bytes
                        break
                    trama = buffer[ini + 1:fin]
                    buffer = buffer[fin + 1:]
                    self._parse(trama)

            except Exception as e:
                print(f"[ULTRASONIDO] Error leyendo serial: {e}")
                self.conectado = False
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(2.0)

    def _parse(self, trama: str):
        """
        Convierte una trama 'a,b,c,d' a floats y actualiza las distancias
        (cm) bajo lock. Tramas malformadas se descartan sin error.
        """
        partes = trama.split(",")
        if len(partes) != len(self.orden):
            return
        try:
            valores = [float(p) for p in partes]
        except ValueError:
            return

        with self._lock:
            for nombre, valor in zip(self.orden, valores):
                self._distancias[nombre] = valor
            self._ts_ultima = time.time()

    def get(self) -> dict:
        """Devuelve una copia de las ultimas distancias + estado."""
        with self._lock:
            distancias = dict(self._distancias)
            ts = self._ts_ultima

        # Se considera "fresco" si llego una trama en los ultimos 2 s
        fresco = (time.time() - ts) < 2.0 if ts > 0 else False

        return {
            "conectado": bool(self.conectado and fresco),
            "distancias": distancias,
            "stop_cm": OBSTACULO_STOP_CM,
            "warn_cm": OBSTACULO_WARN_CM,
        }

    def bloquea(self, nombre: str) -> bool:
        """True si el sensor 'nombre' ve un obstaculo por debajo del umbral STOP de esa cara."""
        with self._lock:
            valor = self._distancias.get(nombre)
        umbral = OBSTACULO_STOP_CM.get(nombre)
        return valor is not None and umbral is not None and valor < umbral


# ---------- PINES BTS7960 ----------
# Motor izquierdo
L_RPWM = 12
L_LPWM = 13
L_R_EN = 23
L_L_EN = 24

# Motor derecho
R_RPWM = 18
R_LPWM = 19
R_R_EN = 25
R_L_EN = 26

# ---------- BALIZA (LUZ + ZUMBADOR) ----------
# Cada uno con su propio MOSFET/GPIO para poder tenerlos independientes:
#   - Luz fija cuando la teleoperacion esta ARMADA (aunque el robot este quieto).
#   - Luz parpadeando + zumbador sonando cuando el robot se MUEVE.
# Si solo tienes un GPIO combinado, pon BALIZA_BUZZER_PIN = BALIZA_LUZ_PIN
# (el zumbador sonara tambien al armar; no recomendado).
BALIZA_LUZ_PIN = 16
BALIZA_BUZZER_PIN = 16
BALIZA_BLINK_HZ = 1  # parpadeos por segundo al moverse

PWM_FREQ = 1000


class BalizaController:
    """
    Controla la baliza en un hilo, con tres estados:
      'off'    -> teleoperacion desarmada: luz y zumbador apagados
      'armed'  -> armada y quieta: luz FIJA, zumbador en silencio
      'moving' -> en movimiento: luz PARPADEANDO + zumbador sonando
    """

    def __init__(self, luz_pin, buzzer_pin):
        """
        Args:
            luz_pin (int): GPIO BCM de la luz.
            buzzer_pin (int): GPIO BCM del zumbador (igual a luz_pin si es combinado).
        """
        self.luz_pin = luz_pin
        self.buzzer_pin = buzzer_pin
        self._state = "off"
        self._lock = threading.Lock()
        self._running = False

        GPIO.setup(self.luz_pin, GPIO.OUT)
        GPIO.output(self.luz_pin, GPIO.LOW)
        if self.buzzer_pin is not None and self.buzzer_pin != self.luz_pin:
            GPIO.setup(self.buzzer_pin, GPIO.OUT)
            GPIO.output(self.buzzer_pin, GPIO.LOW)

    def _luz(self, on):
        """Enciende/apaga la luz (on: bool)."""
        GPIO.output(self.luz_pin, GPIO.HIGH if on else GPIO.LOW)

    def _buzzer(self, on):
        """Enciende/apaga el zumbador (on: bool)."""
        if self.buzzer_pin is None:
            return
        GPIO.output(self.buzzer_pin, GPIO.HIGH if on else GPIO.LOW)

    def start(self):
        """Arranca el hilo de control de la baliza (daemon)."""
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        """Detiene el hilo y apaga luz y zumbador."""
        self._running = False
        try:
            self._luz(False)
            self._buzzer(False)
        except Exception:
            pass

    def set_state(self, state: str):
        """Fuerza un estado ('off'|'armed'|'moving') de forma directa."""
        with self._lock:
            self._state = state

    def set_armed(self, armed: bool):
        # No baja desde 'moving' aqui; solo alterna off <-> armed.
        """Alterna off <-> armed según armed (bool); no interfiere con 'moving'."""
        with self._lock:
            if armed and self._state == "off":
                self._state = "armed"
            elif not armed:
                self._state = "off"

    def set_moving(self, moving: bool):
        """Marca movimiento (moving: bool). Sin efecto si está desarmada."""
        with self._lock:
            if self._state == "off":
                return  # desarmado: no se mueve la baliza
            self._state = "moving" if moving else "armed"

    def _loop(self):
        """
        Hilo de la baliza: aplica el patrón del estado actual
        (off: apagado | armed: luz fija | moving: parpadeo + zumbador).
        """
        half = 1.0 / (2.0 * BALIZA_BLINK_HZ)
        blink_on = False
        while self._running:
            with self._lock:
                st = self._state
            if st == "off":
                self._luz(False)
                self._buzzer(False)
                time.sleep(0.1)
            elif st == "armed":
                self._luz(True)
                self._buzzer(False)
                time.sleep(0.1)
            else:  # moving
                blink_on = not blink_on
                self._luz(blink_on)
                self._buzzer(True)
                time.sleep(half)


class BTS7960Motor:
    """
    Un motor DC manejado por un driver BTS7960 (2 pines PWM + 2 enables).

    Args del constructor:
        rpwm_pin, lpwm_pin (int): pines BCM de PWM adelante/atrás.
        r_en_pin, l_en_pin (int): pines de habilitación (quedan en HIGH).
        invert (bool): invierte el sentido (motor montado en espejo).
    """
    def __init__(self, rpwm_pin, lpwm_pin, r_en_pin, l_en_pin, invert=False):
        """
        Configura los 4 pines del driver, habilita los enables y arranca
        ambos canales PWM a 0 % (PWM_FREQ = 1 kHz).
        """
        self.rpwm_pin = rpwm_pin
        self.lpwm_pin = lpwm_pin
        self.r_en_pin = r_en_pin
        self.l_en_pin = l_en_pin
        self.invert = invert

        GPIO.setup(self.rpwm_pin, GPIO.OUT)
        GPIO.setup(self.lpwm_pin, GPIO.OUT)
        GPIO.setup(self.r_en_pin, GPIO.OUT)
        GPIO.setup(self.l_en_pin, GPIO.OUT)

        GPIO.output(self.r_en_pin, GPIO.HIGH)
        GPIO.output(self.l_en_pin, GPIO.HIGH)

        self.pwm_r = GPIO.PWM(self.rpwm_pin, PWM_FREQ)
        self.pwm_l = GPIO.PWM(self.lpwm_pin, PWM_FREQ)

        self.pwm_r.start(0)
        self.pwm_l.start(0)

    def set_speed(self, speed: float):
        """
        Aplica velocidad con signo: >0 adelante, <0 atrás, 0 detiene.

        Args:
            speed (float): duty PWM en % [-100, 100].
        """
        speed = clamp_pwm(speed)

        if self.invert:
            speed = -speed

        if speed > 0:
            self.pwm_l.ChangeDutyCycle(0)
            self.pwm_r.ChangeDutyCycle(abs(speed))

        elif speed < 0:
            self.pwm_r.ChangeDutyCycle(0)
            self.pwm_l.ChangeDutyCycle(abs(speed))

        else:
            self.pwm_r.ChangeDutyCycle(0)
            self.pwm_l.ChangeDutyCycle(0)

    def stop(self):
        """Detiene el motor (ambos PWM a 0; los enables quedan activos)."""
        self.pwm_r.ChangeDutyCycle(0)
        self.pwm_l.ChangeDutyCycle(0)

    def disable(self):
        """Detiene y deshabilita el driver (enables LOW, PWM liberados)."""
        self.stop()
        GPIO.output(self.r_en_pin, GPIO.LOW)
        GPIO.output(self.l_en_pin, GPIO.LOW)
        self.pwm_r.stop()
        self.pwm_l.stop()


def set_robot_velocity(v_mm_s: float, w_rad_s: float):
    """
    Aplica una consigna (v, w) a ambos motores y actualiza la baliza.

    Args:
        v_mm_s (float), w_rad_s (float): consigna de movimiento.
    Returns:
        dict: consigna, PWM aplicados y estado de la baliza (para el log/API).
    """
    pwm_i, pwm_d = twist_to_pwm(v_mm_s, w_rad_s)

    motor_i.set_speed(pwm_i)
    motor_d.set_speed(pwm_d)

    en_movimiento = (abs(pwm_i) > 0.0) or (abs(pwm_d) > 0.0)
    baliza.set_moving(en_movimiento)

    result = {
        "v_mm_s": v_mm_s,
        "w_rad_s": w_rad_s,
        "pwm_i": pwm_i,
        "pwm_d": pwm_d,
        "baliza": en_movimiento
    }

    print(result)
    return result


def stop_robot():
    """
    Detiene ambos motores, apaga el modo 'moving' de la baliza y
    devuelve el dict de estado con todo en cero.
    """
    motor_i.stop()
    motor_d.stop()

    baliza.set_moving(False)

    result = {
        "v_mm_s": 0.0,
        "w_rad_s": 0.0,
        "pwm_i": 0.0,
        "pwm_d": 0.0,
        "baliza": False
    }

    print(result)
    return result


# ---------- INICIALIZACION GPIO ----------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Estado de armado de la teleoperacion (se activa desde /teleop)
armed = False

# Controlador de baliza (configura sus pines y arranca su hilo)
baliza = BalizaController(BALIZA_LUZ_PIN, BALIZA_BUZZER_PIN)
baliza.start()

motor_i = BTS7960Motor(
    L_RPWM,
    L_LPWM,
    L_R_EN,
    L_L_EN,
    invert=False
)

motor_d = BTS7960Motor(
    R_RPWM,
    R_LPWM,
    R_R_EN,
    R_L_EN,
    invert=True
)

# Lector de ultrasonidos del Arduino Nano (arranca en su propio hilo)
ultrasonido = UltrasonicReader(NANO_PORT, NANO_BAUD, NANO_ORDEN)
ultrasonido.start()

# ---------- SERVIDOR DE LA BASE ----------
app = Flask(__name__)


@app.route("/base", methods=["POST"])
def base_command():
    """
    POST /base — ejecuta un comando de movimiento.

    Body JSON: {'cmd': 'fwd'|'bwd'|'left'|'right'|'stop'}.
    Rechaza con 409 si la teleop no está armada o si el ultrasonido de esa
    dirección detecta obstáculo (fwd/bwd). Responde los PWM aplicados.
    """

    print("\n==============================")
    print("PETICION RECIBIDA DESDE LA PC")

    data = request.get_json()

    print(f"JSON recibido: {data}")

    cmd = data.get("cmd", "").lower()

    print(f"Comando recibido: {cmd}")

    # Puerta de armado: los movimientos solo se ejecutan si la teleoperacion
    # esta armada. 'stop' siempre se permite por seguridad.
    if cmd in ("fwd", "bwd", "left", "right") and not armed:
        print("RECHAZADO: teleoperacion no armada")
        return jsonify({
            "ok": False,
            "cmd": cmd,
            "error": "teleoperacion no armada",
            "armed": False,
        }), 409

    if cmd == "fwd":

        print("ACCION: AVANZAR")

        if ultrasonido.bloquea("delantera"):
            print("BLOQUEADO: obstaculo delante")
            stop_robot()
            respuesta = {
                "ok": False,
                "cmd": cmd,
                "bloqueado": True,
                "motivo": "obstaculo_delante",
                "ultrasonido": ultrasonido.get(),
            }
            print("==============================\n")
            return jsonify(respuesta), 409

        result = set_robot_velocity(V_CMD_MM_S, 0.0)

    elif cmd == "bwd":

        print("ACCION: RETROCEDER")

        if ultrasonido.bloquea("trasera"):
            print("BLOQUEADO: obstaculo detras")
            stop_robot()
            respuesta = {
                "ok": False,
                "cmd": cmd,
                "bloqueado": True,
                "motivo": "obstaculo_detras",
                "ultrasonido": ultrasonido.get(),
            }
            print("==============================\n")
            return jsonify(respuesta), 409

        result = set_robot_velocity(-V_CMD_MM_S, 0.0)

    elif cmd == "left":

        print("ACCION: GIRAR IZQUIERDA")

        result = set_robot_velocity(0.0, -W_CMD_RAD_S)

    elif cmd == "right":

        print("ACCION: GIRAR DERECHA")

        result = set_robot_velocity(0.0, W_CMD_RAD_S)

    elif cmd == "stop":

        print("ACCION: STOP")

        result = stop_robot()

    else:

        print("ERROR: comando invalido")

        return jsonify({
            "ok": False,
            "error": "Comando no valido",
            "cmd": cmd
        }), 400

    result["ok"] = True
    result["cmd"] = cmd
    result["ultrasonido"] = ultrasonido.get()

    print(f"PWM izquierdo: {result['pwm_i']}")
    print(f"PWM derecho : {result['pwm_d']}")
    print("==============================\n")

    return jsonify(result)


@app.route("/teleop", methods=["POST"])
def teleop():
    """
    POST /teleop — arma/desarma la teleoperación.

    Body JSON: {'active': bool}. Al desarmar detiene el robot por seguridad.
    """
    global armed
    data = request.get_json(silent=True) or {}
    armed = bool(data.get("active", False))

    print(f"\nTELEOPERACION: {'ARMADA' if armed else 'DESARMADA'}")

    if armed:
        baliza.set_armed(True)          # luz fija
    else:
        stop_robot()                    # por seguridad, detener al desarmar
        baliza.set_armed(False)         # todo apagado

    return jsonify({"ok": True, "armed": armed})


@app.route("/ultrasonido", methods=["GET"])
def get_ultrasonido():
    """GET /ultrasonido — distancias actuales, umbrales y estado del Nano."""
    return jsonify({
        "ok": True,
        **ultrasonido.get()
    })


@app.route("/status", methods=["GET"])
def status():
    """GET /status — estado general de la base (online, armado, ultrasonido)."""
    return jsonify({
        "ok": True,
        "base": "online",
        "armed": armed,
        "ultrasonido": ultrasonido.get()
    })


if __name__ == "__main__":
    try:
        print("Servidor de base activo en puerto 5005")
        app.run(host="0.0.0.0", port=5005)

    finally:
        try:
            ultrasonido.stop()
        except Exception:
            pass

        try:
            baliza.stop()
        except Exception:
            pass

        try:
            motor_i.disable()
            motor_d.disable()
        except Exception:
            pass

        GPIO.cleanup()
        print("GPIO liberado")
