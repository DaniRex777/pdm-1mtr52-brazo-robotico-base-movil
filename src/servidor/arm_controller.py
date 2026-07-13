"""
Módulo de control del xArm5 — arm_controller.py
Lab Procesos Industriales

Requisitos:
  pip install xArm-Python-SDK

Configuración:
  Editar config.py con la IP del controlador xArm5.

Uso desde server.py:
  from arm_controller import ArmController
  arm = ArmController(ip="192.168.1.155")
  arm.connect()
  arm.set_joints([0, 0, 0, 0, 0])
  arm.set_gripper("open")
  arm.emergency_stop()
  arm.disconnect()
"""

import logging
import threading
import time
from typing import List, Optional

logger = logging.getLogger("arm_controller")

# ─────────────────────────────────────────────
# Tabla de errores del xArm5 (códigos SDK UFactory)
# Fuente: xArm Developer Manual
# ─────────────────────────────────────────────
XARM_ERROR_MESSAGES = {
    0:  ("Sin error", "ok"),
    1:  ("Error de motor — joint 1", "power_cycle"),
    2:  ("Error de motor — joint 2", "power_cycle"),
    3:  ("Error de motor — joint 3", "power_cycle"),
    4:  ("Error de motor — joint 4", "power_cycle"),
    5:  ("Error de motor — joint 5", "power_cycle"),
    10: ("Error de seguimiento de trayectoria", "clear"),
    11: ("Error de comunicación del controlador", "reconnect"),
    12: ("Error de checksum de comunicación", "clear"),
    17: ("Colisión detectada — protección activada", "clear"),
    18: ("Colisión forzada de joint", "clear"),
    19: ("Límite de joint superado", "clear"),
    20: ("Velocidad cartesiana excesiva", "clear"),
    21: ("Aceleración cartesiana excesiva", "clear"),
    22: ("Límite articular superado", "clear"),
    23: ("Singularidad cinemática", "clear"),
    24: ("Sin solución cinemática inversa", "clear"),
    25: ("Punto fuera de espacio de trabajo", "clear"),
    26: ("Límite de velocidad de joint", "clear"),
    27: ("Límite de aceleración de joint", "clear"),
    28: ("Límite de jerk de joint", "clear"),
    29: ("Punto de inicio inválido en modo online", "clear"),
    30: ("Buffer de movimiento lleno", "clear"),
    31: ("Emergencia externa activada", "clear"),
    35: ("Movimiento rechazado por el controlador", "clear"),
}

XARM_WARN_MESSAGES = {
    0:  "Sin advertencia",
    11: "Buffer de trayectoria casi lleno",
    12: "Controlador en límite de temperatura",
    13: "Motor en límite de temperatura",
}

def describe_arm_error(error_code: int, warn_code: int) -> dict:
    """Retorna descripción y acción recomendada para un error/warning del brazo."""
    err_msg, err_action = XARM_ERROR_MESSAGES.get(error_code, (f"Error desconocido (código {error_code})", "clear"))
    warn_msg = XARM_WARN_MESSAGES.get(warn_code, f"Warning desconocido (código {warn_code})")
    return {
        "error_code": error_code,
        "warn_code": warn_code,
        "error_msg": err_msg,
        "warn_msg": warn_msg if warn_code != 0 else None,
        "action": err_action,        # "ok" | "clear" | "reconnect" | "power_cycle"
    }

# Intentar importar el SDK de UFactory
try:
    from xarm.wrapper import XArmAPI
    SDK_AVAILABLE = True
except ImportError:
    SDK_AVAILABLE = False
    logger.warning(
        "xArm SDK no encontrado. Corriendo en modo simulación.\n"
        "Instalar con: pip install xArm-Python-SDK"
    )


# ─────────────────────────────────────────────
# Límites de joints del xArm5 (grados)
# Fuente: datasheet UFactory xArm5
# ─────────────────────────────────────────────

JOINT_LIMITS = [
    (-360.0, 360.0),  # J1 — base
    (-110.0, 10.0),   # J2 — hombro
    (-215.0, 5.0),    # J3 — codo
    (-360.0, 360.0),  # J4 — muñeca
    (-97.0, 180.0),   # J5 — efector
]


# Velocidad y aceleración por defecto (°/s y °/s²)
DEFAULT_SPEED = 50
DEFAULT_ACCEL = 200


# ─────────────────────────────────────────────
# Clase principal
# ─────────────────────────────────────────────

class ArmController:
    """
    Controlador del xArm5. Encapsula el SDK de UFactory
    y expone los métodos que necesita el servidor.

    Modos de operación:
      - Con hardware: conecta al controlador real por Ethernet y mantiene
        un watchdog de reconexión automática.
      - Sin hardware: si el SDK no está disponible, corre en modo simulación.
    """

    # Intentos máximos en el handshake inicial antes de lanzar excepción
    CONNECT_MAX_RETRIES = 5
    CONNECT_RETRY_DELAY = 3.0   # segundos entre intentos

    # Intervalo del watchdog de reconexión en background
    WATCHDOG_INTERVAL = 5.0     # segundos

    def __init__(self, ip: str, speed: int = DEFAULT_SPEED, accel: int = DEFAULT_ACCEL):
        """
        Args:
            ip (str): IP del controlador xArm5 (ver config.py).
            speed (int): velocidad por defecto en °/s.
            accel (int): aceleración por defecto en °/s².
        Si el SDK no está disponible, opera en modo simulación.
        """
        self.ip = ip
        self.speed = speed
        self.accel = accel
        self.connected = False
        self.arm = None
        self._simulated_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._gripper_state = "open"
        self.xyz = [200, 0, 200]
        self._velocity_mode_active = False

        # Estado de error actual (se actualiza en cada operación y en el watchdog)
        self.last_error_code: int = 0
        self.last_warn_code: int = 0
        # Último código retornado por el SDK en comandos de movimiento.
        # Algunos estados, como hand alignment requerido, pueden aparecer
        # como código de retorno aunque arm.error_code siga en 0.
        self.last_sdk_code: int = 0
        self.last_sdk_context: str = ""

        # Watchdog de reconexión en background
        self._watchdog_thread: Optional[threading.Thread] = None
        self._watchdog_stop = threading.Event()
        # Callback opcional: el servidor puede registrar una función para
        # ser notificado cuando el estado de conexión cambie.
        # Firma: on_connection_change(connected: bool)
        self.on_connection_change = None

    # ─────────────────────────────────────────
    # Control de velocidad cartesiana (movimiento fluido)
    # ─────────────────────────────────────────

    def start_cartesian_velocity_mode(self) -> bool:
        """
        Activa el modo 5 del xArm (control de velocidad cartesiana).
        Se debe llamar una sola vez antes de enviar velocidades,
        no en cada comando.
        """
        if not SDK_AVAILABLE or not self.arm:
            logger.info("[SIMULACIÓN] modo velocidad cartesiana activado")
            self._velocity_mode_active = True
            return True

        try:
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.motion_enable(True)
            self.arm.set_mode(5)   # Cartesian velocity control
            self.arm.set_state(0)
            time.sleep(0.2)
            self._velocity_mode_active = True
            return True
        except Exception as e:
            logger.error(f"Error activando modo velocidad cartesiana: {e}")
            self._velocity_mode_active = False
            return False

    def move_xyz_velocity(self, vx: float = 0, vy: float = 0, vz: float = 0) -> bool:
        """
        Mueve el TCP a una velocidad cartesiana constante (mm/s) hasta
        que se llame stop_xyz_velocity() o se reciba un nuevo comando.

        Si el modo de velocidad no está activo lo activa primero.
        """
        if not SDK_AVAILABLE or not self.arm:
            logger.info(f"[SIMULACIÓN] velocidad XYZ vx={vx}, vy={vy}, vz={vz}")
            self._velocity_mode_active = True
            return True

        if not self._velocity_mode_active:
            ok = self.start_cartesian_velocity_mode()
            if not ok:
                return False

        try:
            code = self.arm.vc_set_cartesian_velocity(
                [vx, vy, vz, 0, 0, 0],
                is_radian=False,
                is_tool_coord=False,
                duration=0
            )
            self._record_sdk_code(code, "move_xyz_velocity")
            return code == 0
        except Exception as e:
            logger.error(f"Error en move_xyz_velocity: {e}")
            return False

    def stop_xyz_velocity(self) -> bool:
        """
        Detiene el movimiento cartesiano y sale del modo 5.

        Si ya estamos fuera del modo de velocidad (flag interno),
        no manda ningún comando al SDK para evitar el warning
        "mode may be incorrect, mode: 5 (0)".
        """
        if not SDK_AVAILABLE or not self.arm:
            self._velocity_mode_active = False
            return True

        if not self._velocity_mode_active:
            # Ya parado — nada que hacer, evitar el warning del SDK
            return True

        try:
            # Primero enviar velocidad 0 mientras aún estamos en modo 5
            self.arm.vc_set_cartesian_velocity(
                [0, 0, 0, 0, 0, 0],
                is_radian=False,
                is_tool_coord=False,
                duration=0
            )
        except Exception as e:
            logger.warning(f"Error enviando velocidad 0: {e}")

        # Salir del modo 5 y volver a posición
        self._velocity_mode_active = False
        try:
            self.arm.set_mode(0)
            self.arm.set_state(0)
        except Exception as e:
            logger.warning(f"Error saliendo del modo 5: {e}")

        return True

    def exit_velocity_mode(self):
        """
        Sale del modo 5 y vuelve a modo posición (modo 0).
        Llamar después de soltar el botón si luego se usarán
        set_joints / move_xyz por posición, para no mezclar modos.
        """
        self._velocity_mode_active = False
        if not SDK_AVAILABLE or not self.arm:
            return True
        try:
            self.arm.set_mode(0)
            self.arm.set_state(0)
            time.sleep(0.1)
            return True
        except Exception as e:
            logger.error(f"Error saliendo de modo velocidad: {e}")
            return False


    def move_xyz(self, dx=0, dy=0, dz=0):
        """
        Mueve el TCP en modo incremental (posición relativa).

        Args:
            dx, dy, dz (float): desplazamiento en mm por eje.
        Returns:
            bool: True si el xArm aceptó el movimiento.
        """
        if not SDK_AVAILABLE or not self.arm:
            logger.info(f"[SIMULACIÓN] move_xyz dx={dx}, dy={dy}, dz={dz}")
            return True

        if self._velocity_mode_active:
            self.exit_velocity_mode()

        code, pose = self.arm.get_position()
        if code != 0:
            logger.warning(f"No se pudo leer posición TCP. Código: {code}")
            return False

        x, y, z, roll, pitch, yaw = pose[:6]

        target = [
            x + dx,
            y + dy,
            z + dz,
            roll,
            pitch,
            yaw
        ]

        code = self.arm.set_position(
            x=target[0],
            y=target[1],
            z=target[2],
            roll=target[3],
            pitch=target[4],
            yaw=target[5],
            speed=150,
            mvacc=500,
            wait=False
        )
        self._record_sdk_code(code, "move_xyz")

        return code == 0


    def set_joint1(self, angle):
        """Mueve solo J1 (base) al ángulo dado en grados, manteniendo el resto."""

        joints = self.get_joints()
        joints[0] = angle

        self.set_joints(joints)


    def set_joint5(self, angle):
        """Mueve solo J5 (muñeca) al ángulo dado en grados, manteniendo el resto."""

        joints = self.get_joints()
        joints[4] = angle

        self.set_joints(joints)

    # ─────────────────────────────────────────
    # Conexión — Handshake con reintentos
    # ─────────────────────────────────────────

    def connect(self) -> bool:
        """
        Handshake con el controlador xArm5.

        Intenta conectar hasta CONNECT_MAX_RETRIES veces con pausa entre
        intentos. Si todos fallan lanza ConnectionError.

        Después de una conexión exitosa arranca el watchdog de reconexión
        en background que detectará pérdidas y reconectará solo.
        """
        if not SDK_AVAILABLE:
            logger.info(f"[SIMULACIÓN] Conectado a xArm5 en {self.ip}")
            self.connected = True
            self._start_watchdog()
            return True

        last_exc = None
        for attempt in range(1, self.CONNECT_MAX_RETRIES + 1):
            try:
                logger.info(f"Conectando a xArm5 en {self.ip} (intento {attempt}/{self.CONNECT_MAX_RETRIES})...")
                self.arm = XArmAPI(self.ip, baud_checkset=False)
                time.sleep(0.5)

                self.arm.clean_warn()
                self.arm.clean_error()
                self.arm.motion_enable(enable=True)
                self.arm.set_mode(0)
                self.arm.set_state(state=0)
                time.sleep(0.5)

                # Verificar que respondió: leer estado
                code, state = self.arm.get_state()
                if code != 0:
                    raise RuntimeError(f"get_state retornó código {code}")

                self.connected = True
                logger.info(f"xArm5 conectado y listo (intento {attempt}).")
                self._register_error_callback()
                self._notify_connection(True)
                self._start_watchdog()
                return True

            except Exception as e:
                last_exc = e
                logger.warning(f"Intento {attempt} fallido: {e}")
                if self.arm:
                    try:
                        self.arm.disconnect()
                    except Exception:
                        pass
                    self.arm = None
                self.connected = False
                if attempt < self.CONNECT_MAX_RETRIES:
                    logger.info(f"Reintentando en {self.CONNECT_RETRY_DELAY}s...")
                    time.sleep(self.CONNECT_RETRY_DELAY)

        logger.error(
            f"No se pudo conectar al xArm5 tras {self.CONNECT_MAX_RETRIES} intentos. "
            f"Último error: {last_exc}. La interfaz seguirá funcionando; "
            f"se reintentará la conexión en segundo plano."
        )
        self.connected = False
        self._notify_connection(False)

        # Recuperación automática: arranca el watchdog y lanza la reconexión
        # en segundo plano, para que el brazo se conecte solo cuando lo enciendas
        # sin necesidad de reiniciar el servidor.
        self._start_watchdog()
        threading.Thread(
            target=self._try_reconnect,
            name="xarm-cold-reconnect",
            daemon=True,
        ).start()
        return False

    def disconnect(self):
        """Detiene el watchdog y desconecta el brazo de forma segura."""
        self._stop_watchdog()
        if self.arm:
            try:
                self.arm.disconnect()
                logger.info("xArm5 desconectado.")
            except Exception as e:
                logger.warning(f"Error al desconectar xArm5: {e}")
        self.connected = False
        self._notify_connection(False)

    # ─────────────────────────────────────────
    # Watchdog de reconexión en background
    # ─────────────────────────────────────────

    def _start_watchdog(self):
        """Arranca el hilo de watchdog si no está corriendo."""
        if self._watchdog_thread and self._watchdog_thread.is_alive():
            return
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="xarm-watchdog",
            daemon=True
        )
        self._watchdog_thread.start()
        logger.info("Watchdog de reconexión xArm5 iniciado.")

    def _stop_watchdog(self):
        """Detiene el hilo watchdog de reconexión y espera su cierre (2 s máx)."""
        self._watchdog_stop.set()
        if self._watchdog_thread:
            self._watchdog_thread.join(timeout=2.0)
        self._watchdog_thread = None

    def _watchdog_loop(self):
        """
        Hilo daemon que cada WATCHDOG_INTERVAL segundos verifica si el brazo
        sigue respondiendo. Si detecta desconexión intenta reconectar.
        """
        while not self._watchdog_stop.wait(self.WATCHDOG_INTERVAL):
            if not SDK_AVAILABLE:
                continue  # simulación siempre OK

            alive = self._is_arm_alive()
            if not alive and self.connected:
                logger.warning("Watchdog: xArm5 no responde — intentando reconectar...")
                self.connected = False
                self._notify_connection(False)
                self._try_reconnect()

    def _is_arm_alive(self) -> bool:
        """Ping ligero al brazo: lee el estado. Retorna False si falla."""
        if not self.arm:
            return False
        try:
            code, _ = self.arm.get_state()
            return code == 0
        except Exception:
            return False

    def _try_reconnect(self):
        """
        Bucle de reconexión infinito con backoff exponencial
        (máx 30s entre intentos). Se ejecuta desde el watchdog.
        """
        delay = self.CONNECT_RETRY_DELAY
        attempt = 0
        while not self._watchdog_stop.is_set():
            attempt += 1
            logger.info(f"Reconexión xArm5 — intento {attempt} (espera {delay:.0f}s)...")
            try:
                if self.arm:
                    try:
                        self.arm.disconnect()
                    except Exception:
                        pass
                    self.arm = None

                self.arm = XArmAPI(self.ip, baud_checkset=False)
                time.sleep(0.5)
                self.arm.clean_warn()
                self.arm.clean_error()
                self.arm.motion_enable(enable=True)
                self.arm.set_mode(0)
                self.arm.set_state(state=0)
                time.sleep(0.5)

                code, _ = self.arm.get_state()
                if code != 0:
                    raise RuntimeError(f"get_state código {code}")

                self.connected = True
                logger.info(f"xArm5 reconectado exitosamente (intento {attempt}).")
                self._register_error_callback()
                self._notify_connection(True)
                return

            except Exception as e:
                logger.warning(f"Reconexión fallida: {e}. Próximo intento en {delay:.0f}s.")
                self.arm = None
                self._watchdog_stop.wait(delay)
                delay = min(delay * 1.5, 30.0)

    def _notify_connection(self, connected: bool):
        """Llama al callback del servidor si está registrado."""
        if self.on_connection_change:
            try:
                self.on_connection_change(connected)
            except Exception as e:
                logger.warning(f"Error en on_connection_change: {e}")

    def _on_error_warn_changed(self, data):
        """
        Callback del SDK: se dispara en vivo apenas el controlador reporta
        un cambio de error/warning, ANTES de que el socket pueda cerrarse
        (por ejemplo en una parada de emergencia o un error de corriente).

        Sin esto, get_error_info() solo puede leer error_code/warn_code
        mientras el socket esté abierto; si el controlador corta la
        conexión al mismo tiempo que reporta el error (como pasa con
        "Emergencia externa activada"), el error nunca se llega a leer
        por polling y se pierde.
        """
        try:
            err = data.get("error_code", self.last_error_code)
            warn = data.get("warn_code", self.last_warn_code)
            self.last_error_code = err
            self.last_warn_code = warn
            if err != 0:
                logger.warning(f"Error/warn capturado en vivo: error={err}, warn={warn}")
        except Exception as e:
            logger.warning(f"Error en _on_error_warn_changed: {e}")

    def _register_error_callback(self):
        """
        Registra el callback de error/warning en vivo sobre self.arm.
        Se llama tras cada handshake exitoso (connect y _try_reconnect).
        Si la versión del SDK no soporta este callback, se loguea y se
        continúa sin él (no es crítico para el resto del sistema).
        """
        try:
            self.arm.register_error_warn_changed_callback(self._on_error_warn_changed)
        except Exception as e:
            logger.warning(f"No se pudo registrar callback de error/warning: {e}")


    def _record_sdk_code(self, code: int, context: str = "") -> None:
        """Guarda códigos no cero devueltos por llamadas del SDK.

        El caso importante es code=35: en algunas versiones/configuraciones
        del xArm aparece como retorno del comando y no como error_code
        persistente del controlador. Sin este registro, la interfaz ve
        "Sin error" aunque el movimiento haya sido rechazado por requerir
        alineación de mano.
        """
        try:
            code = int(code)
        except Exception:
            return

        if code == 0:
            # No borrar un 35 previo aquí; se borra solo con clear_errors()
            # o cuando el controlador reporta explícitamente otro error.
            return

        self.last_sdk_code = code
        self.last_sdk_context = context

        if code == 35:
            self.last_error_code = 35
            logger.warning(f"xArm requiere alineación de mano. Contexto: {context}")
        else:
            logger.warning(f"SDK xArm retornó código {code}. Contexto: {context}")

    # ─────────────────────────────────────────
    # Gestión de errores del brazo
    # ─────────────────────────────────────────

    def get_error_info(self) -> dict:
        """
        Retorna el estado de error actual del brazo con descripción
        legible y acción recomendada.

        Si no hay conexión activa con el controlador, no se puede leer
        error_code/warn_code en vivo. En ese caso se retorna el último
        error conocido (capturado por el callback en vivo o por la última
        lectura exitosa), marcado explícitamente como "desconectado", en
        vez de reportar falsamente "Sin error".
        """
        if not SDK_AVAILABLE:
            return describe_arm_error(0, 0)

        if not self.arm or not self.connected:
            info = describe_arm_error(self.last_error_code, self.last_warn_code)
            if self.last_error_code != 0:
                info["error_msg"] = f"[Desconectado] Último error conocido: {info['error_msg']}"
            else:
                info["error_msg"] = "[Desconectado] Sin conexión con el controlador — no se puede leer el estado en vivo"
            info["action"] = "reconnect"
            return info

        try:
            err = int(self.arm.error_code)
            warn = int(self.arm.warn_code)

            # Caso especial: align hand puede llegar como código de retorno
            # de set_servo_angle / set_position aunque error_code sea 0.
            if err == 0 and self.last_sdk_code == 35:
                self.last_error_code = 35
                self.last_warn_code = warn
                return describe_arm_error(35, warn)

            self.last_error_code = err
            self.last_warn_code = warn
            if err != 0:
                self.last_sdk_code = 0
                self.last_sdk_context = ""
            return describe_arm_error(err, warn)
        except Exception as e:
            logger.warning(f"Error leyendo error_code: {e}")
            return describe_arm_error(self.last_error_code, self.last_warn_code)

    def clear_errors(self) -> bool:
        """
        Limpia errores y warnings del brazo y lo deja listo para mover.
        Equivale al botón 'Clear Error' de la interfaz UFactory Studio.
        Retorna True si quedó operativo.
        """
        if not SDK_AVAILABLE or not self.arm:
            logger.info("[SIMULACIÓN] clear_errors")
            self.last_error_code = 0
            self.last_warn_code = 0
            self.last_sdk_code = 0
            self.last_sdk_context = ""
            return True
        try:
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.motion_enable(True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            self._velocity_mode_active = False
            self.last_sdk_code = 0
            self.last_sdk_context = ""
            time.sleep(0.3)
            # Verificar que quedó sin error
            err = self.arm.error_code
            self.last_error_code = err
            self.last_warn_code = self.arm.warn_code
            if err == 0:
                logger.info("Errores del xArm5 limpiados exitosamente.")
                return True
            else:
                logger.warning(f"Después de limpiar, error_code={err} persiste.")
                return False
        except Exception as e:
            logger.error(f"Error en clear_errors: {e}")
            return False

    # ─────────────────────────────────────────
    # Control de joints
    # ─────────────────────────────────────────

    def set_joints(self, angles: List[float], speed: int = None, wait: bool = False) -> bool:
        """
        Mueve el brazo a los ángulos de joint indicados.

        Parámetros:
          angles → lista de 5 ángulos en grados [J1, J2, J3, J4, J5]
          speed  → velocidad en °/s (usa DEFAULT_SPEED si no se especifica)
          wait   → si True, bloquea hasta que el movimiento termine

        Retorna True si el comando fue aceptado.
        """
        if not self._validate_joints(angles):
            return False

        spd = speed or self.speed

        if not SDK_AVAILABLE or not self.arm:
            logger.info(f"[SIMULACIÓN] set_joints: {angles} a {spd}°/s")
            self._simulated_joints = list(angles)
            return True

        try:
            # Los comandos articulares requieren modo de posición.
            # El jog XYZ deja el controlador en modo 5; se debe salir antes.
            if self._velocity_mode_active:
                self.stop_xyz_velocity()

            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            time.sleep(0.05)

            code = self.arm.set_servo_angle(
                angle=angles,
                speed=spd,
                mvacc=self.accel,
                wait=wait,
                radius=-1
            )
            self._record_sdk_code(code, "set_joints")
            if code == 0:
                self._simulated_joints = list(angles)
                logger.debug(f"Joints enviados: {angles}")
                return True
            else:
                logger.warning(f"xArm5 retornó código de error: {code}")
                return False

        except Exception as e:
            logger.error(f"Error en set_joints: {e}")
            return False

    def jog_joint(self, index: int, delta: float, speed: int = None) -> bool:
        """Mueve un solo joint desde la posición REAL reportada por UFactory."""
        if index < 0 or index >= 5:
            logger.warning(f"Índice de joint inválido: {index}")
            return False
        current = self.get_joints()
        if not current or len(current) < 5:
            logger.warning("No se pudo obtener la pose real para el jog del joint")
            return False
        target = list(current[:5])
        lo, hi = JOINT_LIMITS[index]
        target[index] = max(lo, min(hi, target[index] + float(delta)))
        return self.set_joints(target, speed=speed, wait=False)

    def get_joints(self) -> List[float]:
        """
        Retorna los ángulos actuales de los joints en grados.
        """
        if not SDK_AVAILABLE or not self.arm:
            return self._simulated_joints

        try:
            code, angles = self.arm.get_servo_angle()
            if code == 0:
                return list(angles[:5])
            else:
                self._record_sdk_code(code, "get_joints")
                logger.warning(f"Error leyendo joints: código {code}")
                return self._simulated_joints
        except Exception as e:
            logger.error(f"Error en get_joints: {e}")
            return self._simulated_joints

    def go_initial_position(self, wait: bool = True) -> bool:
        """
        Lleva el brazo a la Initial Position configurada
        directamente en UFactory Studio.
        """
        logger.info("Moviendo brazo a la Initial Position de UFactory...")

        if not SDK_AVAILABLE or not self.arm:
            logger.info("[SIMULACIÓN] reset")
            return True

        try:
            # Detener cualquier movimiento cartesiano activo
            self.stop_xyz_velocity()

            # Recuperar el modo normal de posición
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)

            time.sleep(0.2)

            # Usa la Initial Position guardada en UFactory Studio
            code = self.arm.reset(
                wait=wait
            )

            self._record_sdk_code(code, "reset")

            if code != 0:
                logger.warning(
                    f"xArm rechazó reset con código {code}"
                )
                return False

            logger.info(
                "Brazo enviado a la Initial Position de UFactory."
            )
            return True

        except Exception as e:
            logger.error(
                f"Error al ejecutar la Initial Position: {e}"
            )
            return False

    # ─────────────────────────────────────────
    # Control del gripper
    # ─────────────────────────────────────────

    def set_gripper(self, action: str) -> bool:
        """
        Controla el gripper.

        Parámetros:
          action → "open" para abrir, "close" para cerrar

        Nota: si el gripper es propio (no el de UFactory),
        reemplazar este método con el control correspondiente
        (señal digital, PWM, etc.)
        """
        if action not in ("open", "close"):
            logger.warning(f"Acción de gripper inválida: {action}")
            return False

        self._gripper_state = action

        if not SDK_AVAILABLE or not self.arm:
            logger.info(f"[SIMULACIÓN] Gripper: {action}")
            return True

        try:
            # Gripper de UFactory: posición 850 = abierto, 0 = cerrado
            # Si usan gripper propio, reemplazar estas líneas
            pos = 850 if action == "open" else 0
            code = self.arm.set_gripper_position(pos, wait=False, speed=3000)
            if code == 0:
                logger.info(f"Gripper: {action}")
                return True
            else:
                logger.warning(f"Error en gripper: código {code}")
                return False

        except Exception as e:
            logger.error(f"Error en set_gripper: {e}")
            return False

    def get_gripper_state(self) -> str:
        """Retorna el estado actual del gripper: 'open' o 'close'."""
        return self._gripper_state

    # ─────────────────────────────────────────
    # Parada de emergencia
    # ─────────────────────────────────────────

    def emergency_stop(self):
        """
        Detiene el brazo inmediatamente.
        Usa set_state(4) del SDK que es la parada de emergencia oficial.
        """
        logger.warning("EMERGENCIA: deteniendo xArm5")
        self._velocity_mode_active = False

        if not SDK_AVAILABLE or not self.arm:
            logger.info("[SIMULACIÓN] Emergency stop activado")
            return

        try:
            # Estado 4 = parada de emergencia en el SDK de UFactory
            self.arm.set_state(4)
            logger.info("xArm5 detenido por emergencia.")
        except Exception as e:
            logger.error(f"Error en emergency_stop: {e}")

    def resume_after_emergency(self) -> bool:
        """
        Reanuda el brazo después de una parada de emergencia.
        Limpia errores y vuelve al estado normal.
        """
        if not SDK_AVAILABLE or not self.arm:
            logger.info("[SIMULACIÓN] Reanudando tras emergencia")
            self._velocity_mode_active = False
            return True

        try:
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            self._velocity_mode_active = False
            time.sleep(0.5)
            logger.info("xArm5 reanudado.")
            return True
        except Exception as e:
            logger.error(f"Error reanudando xArm5: {e}")
            return False

    # ─────────────────────────────────────────
    # Estado y diagnóstico
    # ─────────────────────────────────────────

    def get_status(self) -> dict:
        """
        Retorna un dict con el estado actual del brazo, incluyendo
        descripción legible del error y acción recomendada.
        Útil para el endpoint /status del servidor.
        """
        error_info = self.get_error_info()
        status = {
            "connected": self.connected,
            "joints": self.get_joints(),
            "gripper": self._gripper_state,
            "error_code": error_info["error_code"],
            "warn_code": error_info["warn_code"],
            "error_msg": error_info["error_msg"],
            "warn_msg": error_info["warn_msg"],
            "error_action": error_info["action"],
            "state": 0,
        }

        if SDK_AVAILABLE and self.arm:
            try:
                status["state"] = self.arm.state
            except Exception:
                pass

        return status

    # ─────────────────────────────────────────
    # Validación interna
    # ─────────────────────────────────────────

    def _validate_joints(self, angles: List[float]) -> bool:
        """
        Verifica que los ángulos estén dentro de los límites del xArm5.
        Retorna False y loga el error si algún joint está fuera de rango.
        """
        if len(angles) != 5:
            logger.error(f"Se esperan 5 joints, se recibieron {len(angles)}")
            return False

        for i, (angle, (lo, hi)) in enumerate(zip(angles, JOINT_LIMITS)):
            if not (lo <= angle <= hi):
                logger.error(
                    f"Joint J{i+1} fuera de rango: {angle}° "
                    f"(límites: {lo}° a {hi}°)"
                )
                return False

        return True