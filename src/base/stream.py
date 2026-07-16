"""
stream.py — Servidor de cámaras de la base (Raspberry Pi)
Lab Procesos Industriales

Sirve varias cámaras USB a la vez para teleoperación:
  - Un hilo de captura por cámara (una cámara lenta o caída no bloquea al resto).
  - Un endpoint MJPEG por cámara:  /video/<id>
  - Página de prueba con todas las cámaras:  /
  - Estado en JSON:  /status

Probar el hardware directamente (sin la interfaz):
  http://<IP-de-la-Pi>:5000/            → todas las cámaras
  http://<IP-de-la-Pi>:5000/video/0     → solo la frontal
"""

import os
import re
import time
import threading

import cv2
from flask import Flask, Response, jsonify

# ─────────────────────────────────────────────
# CONFIGURACIÓN DE CÁMARAS
#
# "device" es el índice de /dev/video*. OJO: cada cámara USB suele ocupar
# DOS índices (video0 y video1 son la misma cámara), así que normalmente
# las cámaras distintas quedan en 0, 2, 4...  Verifica con:
#     v4l2-ctl --list-devices
# y ajusta los índices de abajo según lo que te salga.
# ─────────────────────────────────────────────
# ─────────────────────────────────────────────
# CONFIGURACIÓN DE CÁMARAS
#
# "device" puede ser un índice numérico (0, 2, 4...) o —RECOMENDADO— una
# ruta ESTABLE de /dev/v4l/by-id/. Las rutas by-id no cambian entre reinicios
# y apuntan siempre a la misma cámara física, a diferencia de los números
# /dev/videoN que pueden reordenarse.
#
# Estas rutas salen de:  ls /dev/v4l/by-id/
# Se usa el nodo de CAPTURA de cada cámara (video-index0 / index2), no los
# de metadata (index1 / index3).
# ─────────────────────────────────────────────
BYPATH = "/dev/v4l/by-path"
_P = "platform-fd500000.pcie-pci-0000:01:00.0-usb-0"  # controlador USB de la Pi 4
CAMERAS = [
    # Frontal (recuadro grande) = cámara física de adelante = C170, puerto 1.1.4
    {"id": 0, "name": "Frontal",
     "device": f"{BYPATH}/{_P}:1.1.4:1.0-video-index0"},
    # Izquierda = cámara física de la izquierda = FLH (Integrated), puerto 1.2
    {"id": 2, "name": "Izquierda",
     "device": f"{BYPATH}/{_P}:1.2:1.0-video-index0"},
    # Derecha = cámara física de la derecha = Sonix (USB Live), puerto 1.1.1
    {"id": 1, "name": "Derecha",
     "device": f"{BYPATH}/{_P}:1.1.1:1.0-video-index0"},
]

# Resolución y FPS por cámara. Bajas a propósito para aliviar el bus USB
# (las dos Sonix comparten un puerto). Súbelas si te sobra ancho de banda.
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CAPTURE_FPS = 15
JPEG_QUALITY = 65  # 0-100; menos = menos ancho de banda

PORT = 5000


class CameraWorker:
    """Captura una cámara en un hilo y mantiene siempre el último frame JPEG."""

    def __init__(self, cam_id: int, name: str, device: int):
        """
        Args:
            cam_id (int): id lógico de la cámara (usado en /video/<id>).
            name (str): etiqueta legible ('Frontal', 'Izquierda'...).
            device (int|str): índice /dev/videoN o ruta estable by-path/by-id.
        """
        self.cam_id = cam_id
        self.name = name
        self.device = device

        self._lock = threading.Lock()
        self._latest_jpeg = None
        self._ok = False
        self._running = False
        self._cap = None

    @property
    def ok(self) -> bool:
        """bool: True si la cámara está entregando frames."""
        return self._ok

    def start(self):
        """Arranca el hilo de captura (daemon)."""
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        """Detiene el hilo y libera la cámara."""
        self._running = False
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass

    def _open(self) -> bool:
        # Resolver ruta by-path/by-id (symlink) a su /dev/videoN real y
        # extraer el INDICE NUMERICO: el backend V4L2 de OpenCV abre de forma
        # fiable con un entero, no con una ruta de texto.
        """
        Abre el dispositivo con V4L2 (resuelve rutas by-path a índice),
        intenta MJPG a 320x240@15 y cae al formato por defecto si no da frames.

        Returns:
            bool: True si la cámara quedó abierta y entregando frames.
        """
        dev = self.device
        if isinstance(dev, str):
            if not os.path.exists(dev):
                print(f"[CAM {self.cam_id}] Ruta no existe: {dev}")
                return False
            real = os.path.realpath(dev)          # p.ej. /dev/video0
            m = re.search(r"/dev/video(\d+)", real)
            if m:
                dev = int(m.group(1))             # -> 0, 2, 4...
                print(f"[CAM {self.cam_id}] '{self.name}' -> {real} (indice {dev})")
            else:
                dev = real

        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            print(f"[CAM {self.cam_id}] No abrió el dispositivo.")
            return False

        # Intento 1: MJPG + resolución (mejor ancho de banda). Se verifica con
        # una lectura real, porque algunas cámaras 'aceptan' MJPG pero luego no
        # entregan frames.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        ok, _ = cap.read()

        if not ok:
            # Intento 2: sin forzar formato (deja el que la cámara traiga).
            print(f"[CAM {self.cam_id}] MJPG no dio frame; probando formato por defecto.")
            cap.release()
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            ok, _ = cap.read()
            if not ok:
                cap.release()
                print(f"[CAM {self.cam_id}] Tampoco dio frame en formato por defecto.")
                return False

        self._cap = cap
        return True

    def _loop(self):
        """
        Hilo de captura: lee frames a CAPTURE_FPS, los comprime a JPEG
        (JPEG_QUALITY) y guarda siempre el último. Se auto-recupera si la
        cámara se cae (reintento cada 2 s).
        """
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        period = 1.0 / max(1, CAPTURE_FPS)

        while self._running:
            if self._cap is None or not self._cap.isOpened():
                self._ok = False
                print(f"[CAM {self.cam_id}] Abriendo '{self.name}' ({self.device})...")
                if not self._open():
                    print(f"[CAM {self.cam_id}] No se pudo abrir. Reintentando en 2 s.")
                    time.sleep(2.0)
                    continue
                print(f"[CAM {self.cam_id}] '{self.name}' abierta.")

            t0 = time.time()
            success, frame = self._cap.read()

            if not success:
                self._ok = False
                print(f"[CAM {self.cam_id}] Lectura fallida, reiniciando cámara.")
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None
                time.sleep(0.5)
                continue

            ok, buffer = cv2.imencode(".jpg", frame, encode_params)
            if ok:
                with self._lock:
                    self._latest_jpeg = buffer.tobytes()
                    self._ok = True

            # Cadencia de captura
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)

    def get_jpeg(self):
        """
        Returns:
            bytes|None: último frame JPEG disponible (None si aún no hay).
        """
        with self._lock:
            return self._latest_jpeg


# ─────────────────────────────────────────────
# Arranque de los workers
# ─────────────────────────────────────────────
workers = {}
for cfg in CAMERAS:
    w = CameraWorker(cfg["id"], cfg["name"], cfg["device"])
    w.start()
    workers[cfg["id"]] = w


app = Flask(__name__)


@app.after_request
def add_cors(resp):
    # Permite que la interfaz (en otra IP/puerto) inspeccione los frames
    # para el watchdog anti-congelamiento.
    """Agrega CORS (*) para que la interfaz pueda inspeccionar los frames."""
    resp.headers["Access-Control-Allow-Origin"] = "*"
    return resp


def gen_frames(cam_id: int):
    """
    Generador MJPEG: entrega el último JPEG de la cámara cam_id (int)
    con el boundary multipart, a máx. ~25 fps de salida.
    """
    worker = workers.get(cam_id)
    boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
    while True:
        jpeg = worker.get_jpeg() if worker else None
        if jpeg is not None:
            yield boundary + jpeg + b"\r\n"
        # ~25 fps de salida como máximo
        time.sleep(0.04)


@app.route("/video/<int:cam_id>")
def video(cam_id):
    """GET /video/<cam_id> — stream MJPEG de una cámara (404 si no existe)."""
    if cam_id not in workers:
        return jsonify({"ok": False, "error": "cámara no existe", "cam_id": cam_id}), 404
    return Response(
        gen_frames(cam_id),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# Compatibilidad con el endpoint viejo /video → primera cámara
@app.route("/video")
def video_legacy():
    """GET /video — compatibilidad: stream de la primera cámara (la frontal)."""
    first = CAMERAS[0]["id"]
    return Response(
        gen_frames(first),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/status")
def status():
    """GET /status — JSON con id, nombre, dispositivo y estado de cada cámara."""
    return jsonify({
        "ok": True,
        "cameras": [
            {"id": c["id"], "name": c["name"], "device": c["device"],
             "online": workers[c["id"]].ok}
            for c in CAMERAS
        ],
    })


@app.route("/")
def index():
    """GET / — página de prueba con todas las cámaras en una grilla."""
    cards = "".join(
        f"""
        <div style="background:#111;border-radius:8px;padding:8px;">
          <div style="color:#ccc;font-family:sans-serif;font-size:13px;margin-bottom:6px;">
            [{c['id']}] {c['name']}
          </div>
          <img src="/video/{c['id']}" style="width:100%;border-radius:6px;background:#000;">
        </div>
        """
        for c in CAMERAS
    )
    return f"""
    <html><head><title>Cámaras de la base</title></head>
    <body style="background:#0a0e18;margin:0;padding:16px;">
      <h1 style="color:#eee;font-family:sans-serif;">Cámaras de la base</h1>
      <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px;">
        {cards}
      </div>
    </body></html>
    """


if __name__ == "__main__":
    try:
        print(f"Servidor de cámaras activo en puerto {PORT}")
        # threaded=True: cada stream/cliente se atiende en su propio hilo
        app.run(host="0.0.0.0", port=PORT, threaded=True)
    finally:
        for w in workers.values():
            w.stop()
