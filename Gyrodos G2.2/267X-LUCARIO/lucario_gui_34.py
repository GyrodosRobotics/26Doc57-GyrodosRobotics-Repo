#!/usr/bin/env python3
"""
26Cod71b
LUCARIO GUI [ver 34] v2.1 — Display-only frontend
Subscribes to state from lucario_core, sends commands back via ZMQ.

Keyboard override keys (any tab):
  Z = Force Online (forbid auto-shutdown)
  X = Automatic (let Arduino decide)
  C = Force Offline (manual cut all modules)

GYRODOS ROBOTICS — DESIGNED BY KEANU
"""

import sys
import os
import math
import time
import json
import threading

import zmq

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QFrame, QSizePolicy,
    QStackedWidget, QProgressBar, QCheckBox, QGroupBox,
    QComboBox, QSpinBox, QDoubleSpinBox, QFileDialog,
    QScrollArea, QMessageBox, QOpenGLWidget,
)
from PyQt5.QtCore import Qt, QTimer, QPointF, pyqtSignal, QThread
from PyQt5.QtGui import (
    QPainter, QBrush, QColor, QPen, QFont, QPolygonF,
    QPainterPath, QImage, QPixmap, QKeyEvent,
)

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    import numpy as np;  HAS_GL = True
except ImportError:
    HAS_GL = False

try:
    from stl import mesh as stl_mesh;  HAS_STL = True
except ImportError:
    HAS_STL = False

try:
    import cv2;  HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


# ═══════════════════════════════════════════════════════════════
#  CONNECTION CONFIG
# ═══════════════════════════════════════════════════════════════
CORE_HOST  = "localhost"
STATE_PORT = 5555
CAM_PORT   = 5556
CMD_PORT   = 5557


# ═══════════════════════════════════════════════════════════════
#  COLOUR PALETTE
# ═══════════════════════════════════════════════════════════════
class C:
    BG      = QColor(17, 24, 39)
    PANEL   = QColor(31, 41, 55)
    CTRL    = QColor(55, 65, 81)
    BORDER  = QColor(75, 85, 99)
    DIM     = QColor(156, 163, 175)
    AMBER   = QColor(251, 191, 36)
    RED     = QColor(239, 68, 68)
    GREEN   = QColor(16, 185, 129)
    BLUE    = QColor(96, 165, 250)
    BLUE_B  = QColor(59, 130, 246)
    ORANGE  = QColor(249, 115, 22)
    SKY     = QColor(56, 189, 248)
    GROUND  = QColor(180, 83, 9)
    CYAN    = QColor(6, 182, 212)


# ═══════════════════════════════════════════════════════════════
#  SHUTDOWN BITMASK LABELS
# ═══════════════════════════════════════════════════════════════
SHUTDOWN_NAMES = [
    "Claw L Srv",       # bit 0
    "Claw L Stp",       # bit 1
    "Claw R Srv",       # bit 2
    "Claw R Stp",       # bit 3
    "Cont. Servo",      # bit 4
    "Lights",           # bit 5
]

OVERRIDE_LABELS = {
    0x00: "KEEP ONLINE",
    0x88: "AUTOMATIC",
    0xFF: "FORCE OFFLINE",
}


# ═══════════════════════════════════════════════════════════════
#  LOCAL STATE MIRROR
# ═══════════════════════════════════════════════════════════════
class ROVStateMirror:
    """Thread-safe read-only mirror of core state."""

    _DEFAULTS = dict(
        pitch=0.0, roll=0.0, heading=0.0,
        depth=0.0, speed=0.0, vert_speed=0.0,
        temperature=0.0, pressure=0.0, amperage=0.0,
        thruster_val=[0]*8, thruster_powered=[True]*8,
        thruster_inverted=[False]*8, thruster_inop=[False]*8,
        thruster_trim=[0.0]*8,
        left_stepper_deg=0.0, right_stepper_deg=0.0,
        left_servo_deg=0.0, right_servo_deg=0.0,
        accel_x=0.0, accel_y=0.0, accel_z=0.0,
        distance=0.0, measuring=False,
        autostab=False, false_swipe=False, lights=False,
        emergency=False,
        # v2.1 — power management
        throttle_percent=0,
        shutdown_bitmask=0,
        override_mode=0x88,
    )

    def __init__(self):
        self._data = dict(self._DEFAULTS)
        self._lock = threading.Lock()

    def update(self, state_dict: dict):
        with self._lock:
            self._data.update(state_dict)

    def __getattr__(self, name):
        if name.startswith('_'):
            return super().__getattribute__(name)
        with self._lock:
            return self._data.get(name, 0)


# ═══════════════════════════════════════════════════════════════
#  ZMQ STATE SUBSCRIBER THREAD
# ═══════════════════════════════════════════════════════════════
class StateSubscriber(QThread):
    state_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(bool)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self._running = True

    def run(self):
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://{self.host}:{self.port}")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")
        sub.setsockopt(zmq.RCVTIMEO, 1000)

        connected = False
        while self._running:
            try:
                msg = sub.recv_string()
                data = json.loads(msg)
                self.state_received.emit(data)
                if not connected:
                    connected = True
                    self.connection_changed.emit(True)
            except zmq.Again:
                if connected:
                    connected = False
                    self.connection_changed.emit(False)
            except Exception as e:
                print(f"[GUI] State recv error: {e}")

        sub.close()
        ctx.term()

    def stop(self):
        self._running = False


# ═══════════════════════════════════════════════════════════════
#  ZMQ CAMERA SUBSCRIBER THREAD
# ═══════════════════════════════════════════════════════════════
class CameraSubscriber(QThread):
    frame_received = pyqtSignal(int, QPixmap)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self._running = True
        self._subscribed: set = set()
        self._sub_lock = threading.Lock()

    def subscribe_camera(self, idx: int):
        with self._sub_lock:
            self._subscribed.add(idx)

    def unsubscribe_camera(self, idx: int):
        with self._sub_lock:
            self._subscribed.discard(idx)

    def run(self):
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect(f"tcp://{self.host}:{self.port}")
        sub.setsockopt(zmq.RCVTIMEO, 100)

        for i in range(8):
            sub.setsockopt(zmq.SUBSCRIBE, f"cam:{i}".encode())

        while self._running:
            try:
                topic, jpg_bytes = sub.recv_multipart()
                cam_idx = int(topic.decode().split(":")[1])

                with self._sub_lock:
                    if cam_idx not in self._subscribed:
                        continue

                if HAS_CV2:
                    import numpy as np
                    arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if frame is not None:
                        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        h, w, ch = rgb.shape
                        qimg = QImage(rgb.data, w, h, ch * w,
                                      QImage.Format_RGB888)
                        px = QPixmap.fromImage(qimg.copy())
                        self.frame_received.emit(cam_idx, px)

            except zmq.Again:
                continue
            except Exception as e:
                print(f"[GUI] Camera recv error: {e}")

        sub.close()
        ctx.term()

    def stop(self):
        self._running = False


# ═══════════════════════════════════════════════════════════════
#  COMMAND SENDER
# ═══════════════════════════════════════════════════════════════
class CommandSender:
    def __init__(self, host, port):
        self._ctx = zmq.Context()
        self._pub = self._ctx.socket(zmq.PUB)
        self._pub.connect(f"tcp://{host}:{port}")

    def send(self, **kwargs):
        self._pub.send_string(json.dumps(kwargs))

    def close(self):
        self._pub.close()
        self._ctx.term()


# ═══════════════════════════════════════════════════════════════
#  UTILITY HELPERS
# ═══════════════════════════════════════════════════════════════
def _panel(title=""):
    f = QFrame()
    f.setFrameStyle(QFrame.Box)
    f.setStyleSheet(
        "QFrame{background:#1f2937;border:1px solid #4b5563;border-radius:8px;}")
    lay = QVBoxLayout(f)
    lay.setContentsMargins(8, 8, 8, 8)
    lay.setSpacing(6)
    if title:
        lbl = QLabel(title)
        lbl.setFont(QFont("Courier New", 11, QFont.Bold))
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet("border:none;color:white;")
        lay.addWidget(lbl)
    return f, lay


def _lbl(text, color=None, size=10):
    lb = QLabel(text)
    c = color.name() if isinstance(color, QColor) else (color or "#ffffff")
    lb.setStyleSheet(f"color:{c};font-size:{size}px;border:none;")
    return lb


# ═══════════════════════════════════════════════════════════════
#  PRIMARY FLIGHT DISPLAY
# ═══════════════════════════════════════════════════════════════
class PFD(QWidget):
    def __init__(self, state: ROVStateMirror, parent=None):
        super().__init__(parent)
        self.s = state
        self.setMinimumSize(340, 260)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        tape_w = 44;  vs_w = 44;  gap = 4
        hz_x = tape_w + gap
        hz_w = w - 2 * tape_w - vs_w - 3 * gap
        hz_h = min(hz_w, h - 30)
        hz_y = (h - hz_h) // 2

        self._tape(p, 0, hz_y, tape_w, hz_h,
                   self.s.speed, -2, 2, 0.5, "SPD", left=True)
        self._horizon(p, hz_x, hz_y, hz_w, hz_h)
        dx = hz_x + hz_w + gap
        self._tape(p, dx, hz_y, tape_w, hz_h,
                   self.s.depth, 0, 500, 50, "DEP", left=False, invert=True)
        vx = dx + tape_w + gap
        self._tape(p, vx, hz_y, vs_w, hz_h,
                   self.s.vert_speed, -4000, 4000, 1000, "V/S", left=False)
        p.end()

    def _horizon(self, p: QPainter, x, y, w, h):
        cx, cy = x + w // 2, y + h // 2
        r = min(w, h) // 2 - 2

        p.save()
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), r, r)
        p.setClipPath(clip)
        p.translate(cx, cy)
        p.rotate(-self.s.roll)
        po = self.s.pitch * 2.0

        p.fillRect(-r * 2, int(-r * 2 + po), r * 4, r * 2, QBrush(C.SKY))
        p.fillRect(-r * 2, int(po), r * 4, r * 2, QBrush(C.GROUND))
        p.setPen(QPen(Qt.white, 2))
        p.drawLine(-r * 2, int(po), r * 2, int(po))

        p.setPen(QPen(Qt.white, 1))
        p.setFont(QFont("Courier New", 7))
        for deg in (-30, -20, -10, 10, 20, 30):
            py = int(-deg * 2.0 + po)
            lw = 30 if abs(deg) % 20 == 0 else 20
            p.drawLine(-lw // 2, py, lw // 2, py)
            p.drawText(-lw // 2 - 22, py + 4, str(abs(deg)))
            p.drawText(lw // 2 + 5, py + 4, str(abs(deg)))
        p.restore()

        p.setPen(QPen(QColor(51, 51, 51), 3));  p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r, r)

        p.save();  p.translate(cx, cy)
        for a in (0, 15, -15, 30, -30, 45, -45):
            p.save();  p.rotate(-a)
            ml = 12 if a % 30 == 0 else 8
            p.setPen(QPen(Qt.white, 2 if a % 30 == 0 else 1))
            p.drawLine(0, -(r + 2), 0, -(r + 2 + ml))
            if a % 30 == 0 and a != 0:
                p.setFont(QFont("Courier New", 7))
                p.drawText(-8, -(r + ml + 4), str(abs(a)))
            p.restore()
        p.setPen(Qt.NoPen);  p.setBrush(QBrush(Qt.white))
        p.drawPolygon(QPolygonF([
            QPointF(0, -(r + 2)), QPointF(-5, -(r - 5)),
            QPointF(5, -(r - 5))]))
        p.restore()

        p.setPen(QPen(QColor(255, 255, 0), 3))
        p.drawLine(cx - 20, cy, cx + 20, cy)
        p.drawLine(cx, cy - 10, cx, cy + 10)

    def _tape(self, p, x, y, w, h, value, lo, hi, step, label,
              left, invert=False):
        p.save()
        p.fillRect(x, y, w, h, QBrush(QColor(0, 0, 0, 230)))
        p.setPen(QPen(QColor(51, 51, 51), 2));  p.drawRect(x, y, w, h)
        p.setClipRect(x, y, w, h)

        cy = y + h // 2
        vis = (hi - lo) / 2 or 1
        sign = 1 if invert else -1

        mark = math.floor((value - vis) / step) * step
        while mark <= value + vis:
            py = cy + sign * (mark - value) * (h / (vis * 2))
            if y <= py <= y + h:
                txt = f"{mark:.0f}" if step >= 1 else f"{mark:.1f}"
                p.setPen(QPen(Qt.white, 1))
                p.setFont(QFont("Courier New", 8))
                if left:
                    p.drawLine(int(x + w - 8), int(py), int(x + w), int(py))
                    p.drawText(int(x + 2), int(py + 4), txt)
                else:
                    p.drawLine(int(x), int(py), int(x + 8), int(py))
                    p.drawText(int(x + 12), int(py + 4), txt)
            mark += step

        p.setClipping(False)
        p.setPen(Qt.NoPen);  p.setBrush(QBrush(QColor(255, 255, 0)))
        if left:
            ar = QPolygonF([QPointF(x + w, cy),
                            QPointF(x + w - 8, cy - 4),
                            QPointF(x + w - 8, cy + 4)])
        else:
            ar = QPolygonF([QPointF(x, cy),
                            QPointF(x + 8, cy - 4),
                            QPointF(x + 8, cy + 4)])
        p.drawPolygon(ar)

        p.setPen(QPen(Qt.white))
        p.setFont(QFont("Courier New", 8, QFont.Bold))
        p.drawText(x, y + h + 14, label)
        p.restore()


# ═══════════════════════════════════════════════════════════════
#  COMPASS
# ═══════════════════════════════════════════════════════════════
class Compass(QWidget):
    def __init__(self, state: ROVStateMirror, parent=None):
        super().__init__(parent)
        self.s = state
        self.setMinimumSize(100, 120)
        self.setMaximumSize(140, 160)

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width();  h = self.height() - 20
        cx, cy = w // 2, h // 2
        r = min(w, h) // 2 - 4

        p.setPen(QPen(C.DIM, 2));  p.setBrush(QBrush(Qt.black))
        p.drawEllipse(QPointF(cx, cy), r, r)
        p.setPen(QPen(QColor(107, 114, 128), 1));  p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), r - 8, r - 8)

        p.save();  p.translate(cx, cy);  p.rotate(-self.s.heading)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(C.RED))
        p.drawPolygon(QPolygonF([
            QPointF(0, -(r - 12)), QPointF(-3, 0), QPointF(3, 0)]))
        p.setBrush(QBrush(QColor(200, 200, 200)))
        p.drawPolygon(QPolygonF([
            QPointF(0, r - 12), QPointF(-3, 0), QPointF(3, 0)]))
        p.setFont(QFont("Courier New", 10, QFont.Bold))
        p.setPen(QPen(Qt.white))
        p.drawText(-4, -(r - 24), "N");  p.drawText(-3, r - 14, "S")
        p.drawText(-(r - 16), 5, "W");   p.drawText(r - 22, 5, "E")
        p.restore()

        p.setPen(QPen(Qt.white))
        p.setFont(QFont("Courier New", 11, QFont.Bold))
        txt = f"{int(self.s.heading)}°"
        tw = p.fontMetrics().horizontalAdvance(txt)
        p.drawText(cx - tw // 2, h + 14, txt)
        p.end()


# ═══════════════════════════════════════════════════════════════
#  VERTICAL THRUSTER BAR
# ═══════════════════════════════════════════════════════════════
class VerticalThrusterBar(QWidget):
    def __init__(self, idx, state, parent=None):
        super().__init__(parent)
        self.idx, self.s = idx, state
        self.setMinimumWidth(28);  self.setMaximumWidth(44)
        self.setMinimumHeight(90)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

    def paintEvent(self, _ev):
        p = QPainter(self);  p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        i = self.idx

        tv = self.s.thruster_val
        tp = self.s.thruster_powered
        ti = self.s.thruster_inverted
        tio = self.s.thruster_inop

        val  = tv[i]  if isinstance(tv, list) and len(tv) > i else 0
        on   = tp[i]  if isinstance(tp, list) and len(tp) > i else True
        inv  = ti[i]  if isinstance(ti, list) and len(ti) > i else False
        inop = tio[i] if isinstance(tio, list) and len(tio) > i else False

        p.setFont(QFont("Courier New", 7))
        if on and not inop:
            txt = f"{val:+d}"
            col = C.BLUE if val >= 0 else C.ORANGE
        else:
            txt = "OFF";  col = C.DIM
        p.setPen(QPen(col))
        tw = p.fontMetrics().horizontalAdvance(txt)
        p.drawText((w - tw) // 2, 10, txt)

        mt, mb = 14, 18
        bx = (w - 16) // 2;  bw = 16
        by = mt;  bh = h - mt - mb
        p.fillRect(bx, by, bw, bh, QBrush(QColor(30, 30, 30)))
        p.setPen(QPen(C.BORDER, 1));  p.drawRect(bx, by, bw, bh)

        mid_y = by + bh // 2
        p.setPen(QPen(QColor(80, 80, 80), 1))
        p.drawLine(bx, mid_y, bx + bw, mid_y)

        if on and not inop and val != 0:
            fc = C.BLUE_B if val >= 0 else C.ORANGE
            fh = int(abs(val) / 100 * (bh // 2))
            if val >= 0:
                p.fillRect(bx + 1, mid_y - fh, bw - 1, fh, QBrush(fc))
            else:
                p.fillRect(bx + 1, mid_y + 1, bw - 1, fh, QBrush(fc))

        if inv and on and not inop:
            p.setPen(QPen(C.AMBER));  p.setFont(QFont("Courier New", 6))
            p.drawText(bx - 2, by - 1, "↕")

        p.setFont(QFont("Courier New", 8, QFont.Bold))
        p.setPen(QPen(Qt.white if (on and not inop) else C.DIM))
        tag = f"T{i + 1}"
        tw = p.fontMetrics().horizontalAdvance(tag)
        p.drawText((w - tw) // 2, h - 4, tag)
        p.end()


# ═══════════════════════════════════════════════════════════════
#  ARM DISPLAY
# ═══════════════════════════════════════════════════════════════
class ArmDisplay(QWidget):
    def __init__(self, state: ROVStateMirror, parent=None):
        super().__init__(parent)
        self.s = state
        self.setMinimumHeight(100)

    def paintEvent(self, _ev):
        p = QPainter(self);  p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        half = w // 2

        self._arm(p, 8, 4, half - 16, h - 8,
                  "LEFT ARM", self.s.left_stepper_deg, self.s.left_servo_deg)
        p.setPen(QPen(C.BORDER, 1))
        p.drawLine(half, 8, half, h - 8)
        self._arm(p, half + 8, 4, half - 16, h - 8,
                  "RIGHT ARM", self.s.right_stepper_deg, self.s.right_servo_deg)
        p.end()

    def _arm(self, p, x, y, w, h, title, stepper, servo):
        p.setPen(QPen(C.DIM));  p.setFont(QFont("Courier New", 9, QFont.Bold))
        p.drawText(x, y + 14, title)

        p.setPen(QPen(C.CYAN));  p.setFont(QFont("Courier New", 16, QFont.Bold))
        p.drawText(x, y + 38, f"{stepper:.1f}°")
        p.setPen(QPen(C.DIM));  p.setFont(QFont("Courier New", 8))
        p.drawText(x, y + 50, "STEPPER")

        acx = x + w - 30;  acy = y + 30;  ar = 22
        p.setPen(QPen(C.CTRL, 3));  p.setBrush(Qt.NoBrush)
        p.drawArc(int(acx - ar), int(acy - ar), ar * 2, ar * 2, 0, -90 * 16)

        rad = math.radians(servo)
        ex = acx + ar * math.cos(rad)
        ey = acy + ar * math.sin(rad)
        p.setPen(QPen(C.GREEN, 2))
        p.drawLine(int(acx), int(acy), int(ex), int(ey))
        p.setBrush(QBrush(C.GREEN));  p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(acx, acy), 3, 3)

        p.setPen(QPen(C.DIM));  p.setFont(QFont("Courier New", 7))
        p.drawText(int(acx + ar + 2), int(acy + 4), "FWD")
        p.drawText(int(acx - 10), int(acy + ar + 12), "DN")

        direction = "FWD" if servo < 30 else ("MID" if servo < 60 else "DOWN")
        p.setPen(QPen(C.GREEN));  p.setFont(QFont("Courier New", 9))
        p.drawText(x, y + h - 4, f"Servo {servo:.0f}° {direction}")


# ═══════════════════════════════════════════════════════════════
#  CAMERA FEED WIDGET
# ═══════════════════════════════════════════════════════════════
class CameraFeedWidget(QWidget):
    clicked = pyqtSignal(int)

    def __init__(self, quad_idx, parent=None):
        super().__init__(parent)
        self.quad_idx = quad_idx
        self.camera_idx = -1
        self.pixmap = None
        self.selected = False
        self.setMinimumSize(200, 150)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_camera(self, idx):
        self.camera_idx = idx;  self.update()

    def update_frame(self, px):
        self.pixmap = px;  self.update()

    def mousePressEvent(self, ev):
        self.clicked.emit(self.quad_idx)

    def paintEvent(self, _ev):
        p = QPainter(self);  p.setRenderHint(QPainter.SmoothPixmapTransform)
        w, h = self.width(), self.height()

        if self.pixmap and not self.pixmap.isNull():
            sc = self.pixmap.scaled(w, h, Qt.KeepAspectRatio,
                                    Qt.SmoothTransformation)
            p.drawPixmap((w - sc.width()) // 2, (h - sc.height()) // 2, sc)
        else:
            p.fillRect(0, 0, w, h, QBrush(Qt.black))
            p.setPen(QPen(C.DIM));  p.setFont(QFont("Courier New", 12))
            if self.camera_idx >= 0:
                lines = [f"CAM {self.camera_idx + 1}", "No Signal"]
            else:
                lines = ["Click to select", "Press 1‑8 to assign"]
            for i, ln in enumerate(lines):
                tw = p.fontMetrics().horizontalAdvance(ln)
                p.drawText((w - tw) // 2, h // 2 - 8 + i * 22, ln)

        p.setPen(QPen(C.BLUE if self.selected else C.BORDER,
                       3 if self.selected else 1))
        p.setBrush(Qt.NoBrush);  p.drawRect(1, 1, w - 2, h - 2)

        if self.camera_idx >= 0:
            p.fillRect(4, 4, 60, 20, QBrush(QColor(0, 0, 0, 180)))
            p.setPen(QPen(Qt.white))
            p.setFont(QFont("Courier New", 9, QFont.Bold))
            p.drawText(8, 18, f"CAM {self.camera_idx + 1}")
        p.end()


# ═══════════════════════════════════════════════════════════════
#  STL 3-D VIEWER
# ═══════════════════════════════════════════════════════════════
if HAS_GL:
    class STLViewer(QOpenGLWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.verts = self.norms = None
            self.center = [0, 0, 0];  self.scale = 1.0
            self.rx = -60;  self.ry = 0;  self.rz = 0
            self.zoom = 1.0;  self._last = None
            self.setMinimumSize(300, 300)

        def load_stl(self, path):
            if not HAS_STL:
                return False
            try:
                m = stl_mesh.Mesh.from_file(path)
                self.verts = m.vectors;  self.norms = m.normals
                pts = m.vectors.reshape(-1, 3)
                self.center = pts.mean(axis=0)
                self.scale = 2.0 / max(pts.max(axis=0) - pts.min(axis=0))
                self.update();  return True
            except Exception as e:
                print(f"STL load error: {e}");  return False

        def set_rotation(self, rx, ry, rz):
            self.rx, self.ry, self.rz = rx, ry, rz
            self.update()

        def initializeGL(self):
            glClearColor(0.067, 0.094, 0.153, 1)
            glEnable(GL_DEPTH_TEST);  glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0);      glEnable(GL_COLOR_MATERIAL)
            glLightfv(GL_LIGHT0, GL_POSITION, [1, 1, 1, 0])
            glLightfv(GL_LIGHT0, GL_DIFFUSE,  [.8, .8, .8, 1])
            glLightfv(GL_LIGHT0, GL_AMBIENT,  [.25, .25, .25, 1])

        def resizeGL(self, w, h):
            glViewport(0, 0, w, h)
            glMatrixMode(GL_PROJECTION);  glLoadIdentity()
            gluPerspective(45, w / max(h, 1), 0.1, 100)
            glMatrixMode(GL_MODELVIEW)

        def paintGL(self):
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            glTranslatef(0, 0, -5 / self.zoom)
            glRotatef(self.rx, 1, 0, 0)
            glRotatef(self.ry, 0, 1, 0)
            glRotatef(self.rz, 0, 0, 1)

            glDisable(GL_LIGHTING)
            glColor3f(0.1, 0.3, 0.5)
            glBegin(GL_LINES)
            for i in range(-10, 11):
                glVertex3f(i * .2, 0, -2);  glVertex3f(i * .2, 0, 2)
                glVertex3f(-2, 0, i * .2);  glVertex3f(2, 0, i * .2)
            glEnd()
            glEnable(GL_LIGHTING)

            if self.verts is not None:
                glColor3f(0.4, 0.65, 0.95)
                glBegin(GL_TRIANGLES)
                for j, tri in enumerate(self.verts):
                    if self.norms is not None:
                        glNormal3f(*self.norms[j])
                    for v in tri:
                        glVertex3f(*((v - self.center) * self.scale))
                glEnd()

        def mousePressEvent(self, e):  self._last = e.pos()
        def mouseMoveEvent(self, e):
            if self._last:
                dx = e.x() - self._last.x();  dy = e.y() - self._last.y()
                self.ry += dx * .5;  self.rx += dy * .5
                self._last = e.pos();  self.update()
        def wheelEvent(self, e):
            self.zoom *= 1.1 if e.angleDelta().y() > 0 else 0.9
            self.zoom = max(0.1, min(10, self.zoom));  self.update()
else:
    class STLViewer(QWidget):
        def __init__(self, parent=None):
            super().__init__(parent);  self.setMinimumSize(300, 200)
        def load_stl(self, _): return False
        def set_rotation(self, *_): pass
        def paintEvent(self, _):
            p = QPainter(self);  p.fillRect(self.rect(), QBrush(C.PANEL))
            p.setPen(QPen(C.DIM));  p.setFont(QFont("Courier New", 11))
            p.drawText(20, self.height() // 2,
                       "pip install PyOpenGL numpy-stl")
            p.end()


# ═══════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ═══════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.st = ROVStateMirror()
        self.selected_quad = 0

        # ZMQ connections
        self.cmd = CommandSender(CORE_HOST, CMD_PORT)

        self.state_sub = StateSubscriber(CORE_HOST, STATE_PORT)
        self.state_sub.state_received.connect(self._on_state)
        self.state_sub.connection_changed.connect(self._on_connection)
        self.state_sub.start()

        self.cam_sub = CameraSubscriber(CORE_HOST, CAM_PORT)
        self.cam_sub.frame_received.connect(self._on_frame)
        self.cam_sub.start()

        self.setWindowTitle(
            "LUCARIO ROV Control System — GYRODOS ROBOTICS — DESIGNED BY KEANU")
        self.setMinimumSize(1400, 900)
        self._apply_style()
        self._build()
        self._start_timers()

    # ── ZMQ callbacks ──────────────────────────────────────────
    def _on_state(self, data: dict):
        self.st.update(data)

    def _on_connection(self, connected: bool):
        if connected:
            self.conn_lbl.setText("🌐 CONNECTED")
            self.conn_lbl.setStyleSheet("color:#10b981;font-size:12px;")
        else:
            self.conn_lbl.setText("🔴 DISCONNECTED")
            self.conn_lbl.setStyleSheet("color:#ef4444;font-size:12px;")

    def _on_frame(self, cam_idx: int, pixmap: QPixmap):
        idx = self.stack.currentIndex()
        if idx == 0:
            for q in self.quad:
                if q.camera_idx == cam_idx:
                    q.update_frame(pixmap)
        elif idx == 1:
            if self.mixed_cam.camera_idx == cam_idx:
                self.mixed_cam.update_frame(pixmap)

    # ── dark stylesheet ────────────────────────────────────────
    def _apply_style(self):
        self.setStyleSheet("""
            QMainWindow{background:#111827;}
            QWidget{color:white;font-family:'Courier New',monospace;}
            QLabel{color:white;}
            QPushButton{background:#374151;color:white;border:none;
                        border-radius:3px;padding:4px 8px;font-size:10px;}
            QPushButton:hover{background:#4b5563;}
            QPushButton:checked{background:#3b82f6;}
            QComboBox,QSpinBox,QDoubleSpinBox{background:#4b5563;color:white;
                        border:none;border-radius:4px;padding:4px;}
            QGroupBox{color:#fbbf24;border:1px solid #4b5563;border-radius:8px;
                      margin-top:8px;padding-top:16px;font-weight:bold;}
            QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 3px;}
        """)

    # ══════════════════════════════════════════════════════════════
    #  BUILD
    # ══════════════════════════════════════════════════════════════
    def _build(self):
        root = QWidget();  self.setCentralWidget(root)
        ml = QVBoxLayout(root)
        ml.setContentsMargins(12, 8, 12, 8);  ml.setSpacing(6)

        ml.addLayout(self._header())

        # tab buttons
        tr = QHBoxLayout();  tr.setSpacing(0)
        self.tab_btns = []
        for i, name in enumerate(
                ["📷  CAMERAS", "🎮  MIXED VIEW", "⚙️  SETUP"]):
            b = QPushButton(name)
            b.setCheckable(True);  b.setChecked(i == 1)
            b.setFixedHeight(32)
            b.setStyleSheet(
                "QPushButton{background:#1f2937;border:1px solid #4b5563;"
                "font-size:12px;font-weight:bold;padding:4px 20px;"
                "border-radius:6px 6px 0 0;}"
                "QPushButton:checked{background:#3b82f6;border-color:#3b82f6;}"
                "QPushButton:hover:!checked{background:#374151;}")
            b.clicked.connect(lambda _, idx=i: self._tab(idx))
            tr.addWidget(b);  self.tab_btns.append(b)
        tr.addStretch()
        ml.addLayout(tr)

        # stack
        self.stack = QStackedWidget()
        self.stack.addWidget(self._page_cameras())
        self.stack.addWidget(self._page_mixed())
        self.stack.addWidget(self._page_setup())
        self.stack.setCurrentIndex(1)
        ml.addWidget(self.stack, 1)

    def _tab(self, idx):
        for i, b in enumerate(self.tab_btns):
            b.setChecked(i == idx)
        self.stack.setCurrentIndex(idx)

    # ── header ─────────────────────────────────────────────────
    def _header(self):
        h = QHBoxLayout()
        t = QLabel("LUCARIO  —  GYRODOS ROBOTICS  —  DESIGNED BY KEANU")
        t.setFont(QFont("Courier New", 14, QFont.Bold))
        t.setStyleSheet("color:#60a5fa;")
        h.addWidget(t);  h.addStretch()

        self.conn_lbl = QLabel("🔴 DISCONNECTED")
        self.conn_lbl.setStyleSheet("color:#ef4444;font-size:12px;")
        h.addWidget(self.conn_lbl)

        self.time_lbl = QLabel("")
        self.time_lbl.setStyleSheet("font-size:12px;")
        h.addWidget(self.time_lbl)

        es = QPushButton("⛔ E-STOP")
        es.setStyleSheet(
            "QPushButton{background:#dc2626;font-size:14px;font-weight:bold;"
            "padding:8px 16px;border-radius:4px;}"
            "QPushButton:hover{background:#b91c1c;}")
        es.clicked.connect(self._estop)
        h.addWidget(es)
        return h

    # ══════════════════════════════════════════════════════════════
    #  PAGE 0 – CAMERAS
    # ══════════════════════════════════════════════════════════════
    def _page_cameras(self):
        pg = QWidget()
        ly = QVBoxLayout(pg);  ly.setContentsMargins(4, 4, 4, 4)

        tb = QHBoxLayout()
        tb.addWidget(
            _lbl("Click a quadrant → press 1-8 to assign a camera feed.",
                 C.DIM, 11))
        tb.addStretch()

        mb = QPushButton("📏 MEASURE MODE")
        mb.setStyleSheet(
            "QPushButton{background:#059669;font-size:11px;"
            "font-weight:bold;padding:6px 14px;}"
            "QPushButton:hover{background:#047857;}")
        mb.clicked.connect(self._measure_mode)
        tb.addWidget(mb)

        rb = QPushButton("↺ RESET")
        rb.clicked.connect(self._reset_cams)
        tb.addWidget(rb)
        ly.addLayout(tb)

        grid = QGridLayout();  grid.setSpacing(4)
        self.quad = []
        for i in range(4):
            cw = CameraFeedWidget(i)
            cw.clicked.connect(self._sel_quad)
            grid.addWidget(cw, i // 2, i % 2)
            self.quad.append(cw)
        self.quad[0].selected = True
        ly.addLayout(grid, 1)
        return pg

    def _sel_quad(self, idx):
        self.selected_quad = idx
        for i, q in enumerate(self.quad):
            q.selected = (i == idx);  q.update()

    def _measure_mode(self):
        self.quad[0].set_camera(0)
        self.cam_sub.subscribe_camera(0)
        self.cmd.send(action="open_camera", idx=0)
        self.quad[1].set_camera(1)
        self.cam_sub.subscribe_camera(1)
        self.cmd.send(action="open_camera", idx=1)

    def _reset_cams(self):
        for q in self.quad:
            if q.camera_idx >= 0:
                self.cam_sub.unsubscribe_camera(q.camera_idx)
            q.set_camera(-1);  q.pixmap = None;  q.update()

    # ══════════════════════════════════════════════════════════════
    #  PAGE 1 – MIXED VIEW
    # ══════════════════════════════════════════════════════════════
    def _page_mixed(self):
        pg = QWidget()
        ly = QVBoxLayout(pg)
        ly.setContentsMargins(4, 4, 4, 4);  ly.setSpacing(6)

        # ── TOP: camera (7)  |  instruments (3) ──
        top = QHBoxLayout();  top.setSpacing(8)

        cam_f = QFrame()
        cam_f.setStyleSheet(
            "QFrame{background:black;border:1px solid #4b5563;"
            "border-radius:4px;}")
        cl = QVBoxLayout(cam_f);  cl.setContentsMargins(0, 0, 0, 0)
        self.mixed_cam = CameraFeedWidget(-1)
        self.mixed_cam.camera_idx = 0
        cl.addWidget(self.mixed_cam)
        top.addWidget(cam_f, 7)

        # right instruments
        ri = QVBoxLayout();  ri.setSpacing(6)

        af, afl = _panel("ATTITUDE")
        self.pfd = PFD(self.st);  afl.addWidget(self.pfd)
        ri.addWidget(af, 3)

        hr = QHBoxLayout();  hr.setSpacing(6)
        hf, hfl = _panel("HEADING")
        self.compass = Compass(self.st);  hfl.addWidget(self.compass)
        hr.addWidget(hf)

        ef, efl = _panel("ENVT")
        eg = QGridLayout()
        self.dep_lbl = QLabel("0.0 m")
        self.dep_lbl.setStyleSheet("color:#06b6d4;border:none;")
        self.tmp_lbl = QLabel("0.0 °C")
        self.tmp_lbl.setStyleSheet("color:#10b981;border:none;")
        self.prs_lbl = QLabel("0.00 bar")
        self.prs_lbl.setStyleSheet("color:#eab308;border:none;")
        for r, (n, w) in enumerate([
            ("Depth:", self.dep_lbl),
            ("Temp:",  self.tmp_lbl),
            ("Press:", self.prs_lbl),
        ]):
            lb = QLabel(n);  lb.setStyleSheet("border:none;font-size:10px;")
            eg.addWidget(lb, r, 0);  eg.addWidget(w, r, 1)
        efl.addLayout(eg)
        hr.addWidget(ef)
        ri.addLayout(hr, 2)

        top.addLayout(ri, 3)
        ly.addLayout(top, 4)

        # ── MIDDLE: accel/dist  |  cam select  |  amperage ──
        mid = QHBoxLayout();  mid.setSpacing(8)

        adf, adfl = _panel("ACCEL / DIST")
        adr = QHBoxLayout()
        self.acc_lbl = {}
        for ax in ("X", "Y", "Z"):
            vl = QVBoxLayout()
            vl.addWidget(_lbl(ax, C.DIM, 9))
            v = QLabel("0.00");  v.setAlignment(Qt.AlignCenter)
            v.setStyleSheet(
                "color:#06b6d4;font-weight:bold;font-size:11px;border:none;")
            vl.addWidget(v);  adr.addLayout(vl)
            self.acc_lbl[ax] = v
        dc = QVBoxLayout()
        dc.addWidget(_lbl("DIST", C.DIM, 9))
        self.dist_lbl = QLabel("0.0 cm")
        self.dist_lbl.setStyleSheet(
            "color:#fbbf24;font-weight:bold;font-size:11px;border:none;")
        dc.addWidget(self.dist_lbl)
        adr.addLayout(dc)
        adfl.addLayout(adr)

        mr = QHBoxLayout()
        for txt, col, slot in [
            ("START", "#059669", self._meas_start),
            ("STOP",  "#dc2626", self._meas_stop),
            ("RESET", "#6b7280", self._meas_reset),
        ]:
            b = QPushButton(txt);  b.setFixedHeight(22)
            b.setStyleSheet(f"QPushButton{{background:{col};font-size:9px;}}")
            b.clicked.connect(slot);  mr.addWidget(b)
        adfl.addLayout(mr)
        mid.addWidget(adf, 3)

        csf, csfl = _panel("CAM")
        cg = QGridLayout()
        self.mcam_btns = []
        for i in range(8):
            b = QPushButton(f"{i + 1}")
            b.setCheckable(True);  b.setChecked(i == 0)
            b.setFixedSize(32, 26)
            b.clicked.connect(lambda _, idx=i: self._set_mixed_cam(idx))
            cg.addWidget(b, i // 4, i % 4);  self.mcam_btns.append(b)
        csfl.addLayout(cg)
        mid.addWidget(csf, 2)

        ampf, ampfl = _panel("CURRENT  (A)")
        self.amp_lbl = QLabel("0.0 A")
        self.amp_lbl.setAlignment(Qt.AlignCenter)
        self.amp_lbl.setFont(QFont("Courier New", 22, QFont.Bold))
        self.amp_lbl.setStyleSheet("color:#fbbf24;border:none;")
        ampfl.addWidget(self.amp_lbl)
        self.amp_bar = QProgressBar()
        self.amp_bar.setRange(0, 100);  self.amp_bar.setValue(0)
        self.amp_bar.setTextVisible(False);  self.amp_bar.setFixedHeight(8)
        self.amp_bar.setStyleSheet(
            "QProgressBar{background:#374151;border-radius:4px;border:none;}"
            "QProgressBar::chunk{background:#fbbf24;border-radius:4px;}")
        ampfl.addWidget(self.amp_bar)
        mid.addWidget(ampf, 2)

        ly.addLayout(mid, 1)

        # ── BOTTOM: thrusters  |  arms  |  alerts & status ──
        bot = QHBoxLayout();  bot.setSpacing(8)

        tf, tfl = _panel("THRUSTERS")
        thr = QHBoxLayout();  thr.setSpacing(2)
        self.vthr = []
        for i in range(8):
            vb = VerticalThrusterBar(i, self.st)
            thr.addWidget(vb);  self.vthr.append(vb)
        tfl.addLayout(thr)
        bot.addWidget(tf, 3)

        armf, arml = _panel("ARMS")
        self.arm_disp = ArmDisplay(self.st)
        self.arm_disp.setStyleSheet("border:none;")
        arml.addWidget(self.arm_disp)
        bot.addWidget(armf, 2)

        # ── ALERTS & STATUS panel (v2.1 — expanded) ──
        alf, all_ = _panel("ALERTS & STATUS")

        # Override mode indicator
        self.override_lbl = QLabel("⚡ Override: AUTOMATIC")
        self.override_lbl.setStyleSheet(
            "color:#fbbf24;border:none;font-size:10px;font-weight:bold;")
        all_.addWidget(self.override_lbl)

        self.override_hint_lbl = QLabel("[Z] Online  [X] Auto  [C] Offline")
        self.override_hint_lbl.setStyleSheet(
            "color:#6b7280;border:none;font-size:9px;")
        all_.addWidget(self.override_hint_lbl)

        # Throttle indicator
        self.throttle_lbl = QLabel("🔋 Throttle: None")
        self.throttle_lbl.setStyleSheet(
            "color:#10b981;border:none;font-size:10px;")
        all_.addWidget(self.throttle_lbl)

        # Module shutdown indicator
        self.shutdown_lbl = QLabel("🔌 Modules: All Online")
        self.shutdown_lbl.setStyleSheet(
            "color:#10b981;border:none;font-size:10px;")
        all_.addWidget(self.shutdown_lbl)

        # Active special modes
        self.modes_lbl = QLabel("")
        self.modes_lbl.setStyleSheet(
            "color:#6b7280;border:none;font-size:10px;")
        all_.addWidget(self.modes_lbl)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("border:none;border-top:1px solid #4b5563;")
        sep.setFixedHeight(2)
        all_.addWidget(sep)

        # Alerts
        self.alert_lbl = QLabel("No alerts")
        self.alert_lbl.setWordWrap(True)
        self.alert_lbl.setStyleSheet(
            "color:#10b981;border:none;font-size:11px;")
        all_.addWidget(self.alert_lbl)

        all_.addStretch()

        bot.addWidget(alf, 3)

        ly.addLayout(bot, 2)
        return pg

    def _set_mixed_cam(self, idx):
        for i, b in enumerate(self.mcam_btns):
            b.setChecked(i == idx)
        self.mixed_cam.set_camera(idx)
        self.cam_sub.subscribe_camera(idx)
        self.cmd.send(action="open_camera", idx=idx)

    # ══════════════════════════════════════════════════════════════
    #  PAGE 2 – SETUP
    # ══════════════════════════════════════════════════════════════
    def _page_setup(self):
        inner = QWidget()
        lay = QVBoxLayout(inner)
        lay.setContentsMargins(8, 8, 8, 8);  lay.setSpacing(10)

        # thruster config
        tg = QGroupBox("🔧  THRUSTER CONFIGURATION")
        tgl = QGridLayout(tg)
        for c, h in enumerate(["", "Powered", "Inverted", "Trim %"]):
            lb = QLabel(h);  lb.setStyleSheet("font-weight:bold;font-size:11px;")
            lb.setAlignment(Qt.AlignCenter);  tgl.addWidget(lb, 0, c)

        self.s_pwr = [];  self.s_inv = [];  self.s_trim = []
        for i in range(8):
            lb = QLabel(f"T{i + 1}")
            lb.setAlignment(Qt.AlignCenter)
            lb.setFont(QFont("Courier New", 12, QFont.Bold))
            tgl.addWidget(lb, i + 1, 0)

            pw = QCheckBox("ON");  pw.setChecked(True)
            pw.toggled.connect(lambda chk, idx=i: self._sp(idx, chk))
            tgl.addWidget(pw, i + 1, 1, Qt.AlignCenter)
            self.s_pwr.append(pw)

            iv = QCheckBox("INV");  iv.setChecked(False)
            iv.toggled.connect(lambda chk, idx=i: self._si(idx, chk))
            tgl.addWidget(iv, i + 1, 2, Qt.AlignCenter)
            self.s_inv.append(iv)

            tr = QDoubleSpinBox();  tr.setRange(-100, 100)
            tr.setSingleStep(1);  tr.setValue(0)
            tr.valueChanged.connect(lambda v, idx=i: self._str(idx, v))
            tgl.addWidget(tr, i + 1, 3)
            self.s_trim.append(tr)
        lay.addWidget(tg)

        # mode settings
        mg = QGroupBox("🎛️  MODE SETTINGS")
        mgl = QVBoxLayout(mg)
        mf = QFrame()
        mf.setStyleSheet(
            "QFrame{background:#0f172a;border:1px solid #334155;"
            "border-radius:4px;padding:8px;}")
        mfl = QVBoxLayout(mf)
        self.mode_cbs = {}
        for label, key, default in [
            ("False Swipe Mode", "false_swipe", False),
            ("Autostabilize",    "autostab",    False),
            ("Controls Lock",    "lock",        False),
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            cb = QCheckBox();  cb.setChecked(default)
            cb.toggled.connect(
                lambda chk, k=key: self.cmd.send(
                    action="set_mode", mode=k, value=chk))
            row.addWidget(cb);  row.addStretch()
            mfl.addLayout(row)
            self.mode_cbs[key] = cb
        mgl.addWidget(mf)
        lay.addWidget(mg)

        # 3D model viewer
        vg = QGroupBox("🐟  MILOTIC 3D MODEL VIEWER")
        vgl = QVBoxLayout(vg)

        vc = QHBoxLayout()
        lb = QPushButton("📂 Load .STL")
        lb.setStyleSheet(
            "QPushButton{background:#3b82f6;font-size:11px;padding:6px 14px;}")
        lb.clicked.connect(self._load_stl)
        vc.addWidget(lb)
        self.stl_info = QLabel("No model loaded")
        self.stl_info.setStyleSheet("font-size:10px;color:#9ca3af;border:none;")
        vc.addWidget(self.stl_info)
        vc.addStretch()
        self.stl_rot_lbl = QLabel("Pitch 0°  Yaw 0°  Roll 0°")
        self.stl_rot_lbl.setStyleSheet("font-size:10px;border:none;")
        vc.addWidget(self.stl_rot_lbl)
        vgl.addLayout(vc)

        self.stl_viewer = STLViewer()
        self.stl_viewer.setMinimumHeight(350)
        vgl.addWidget(self.stl_viewer)
        lay.addWidget(vg)

        lay.addStretch()

        scroll = QScrollArea()
        scroll.setWidgetResizable(True);  scroll.setWidget(inner)
        scroll.setStyleSheet("QScrollArea{border:none;background:#111827;}")
        return scroll

    # ── setup command helpers ──────────────────────────────────
    def _sp(self, i, v):
        self.cmd.send(action="set_thruster_power", idx=i, value=v)

    def _si(self, i, v):
        self.cmd.send(action="set_thruster_invert", idx=i, value=v)

    def _str(self, i, v):
        self.cmd.send(action="set_thruster_trim", idx=i, value=float(v))

    def _load_stl(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load STL", "", "STL Files (*.stl);;All (*)")
        if path:
            if self.stl_viewer.load_stl(path):
                self.stl_info.setText(os.path.basename(path))
                self.stl_info.setStyleSheet(
                    "font-size:10px;color:#10b981;border:none;")
            else:
                QMessageBox.warning(
                    self, "Error",
                    "Failed to load STL.\n"
                    "pip install PyOpenGL numpy-stl")

    # ══════════════════════════════════════════════════════════════
    #  KEYBOARD — override keys + camera assignment
    # ══════════════════════════════════════════════════════════════
    def keyPressEvent(self, ev: QKeyEvent):
        key = ev.key()

        # ── Power override keys (global, any tab, no modifiers) ──
        if ev.modifiers() == Qt.NoModifier:
            if key == Qt.Key_Z:
                self.cmd.send(action="set_override", value=0x00)
                return
            elif key == Qt.Key_X:
                self.cmd.send(action="set_override", value=0x88)
                return
            elif key == Qt.Key_C:
                self.cmd.send(action="set_override", value=0xFF)
                return

        # ── Camera page: number keys assign cameras ──
        if self.stack.currentIndex() == 0:
            if Qt.Key_1 <= key <= Qt.Key_8:
                ci = key - Qt.Key_1
                if 0 <= self.selected_quad < 4:
                    self.quad[self.selected_quad].set_camera(ci)
                    self.cam_sub.subscribe_camera(ci)
                    self.cmd.send(action="open_camera", idx=ci)

        super().keyPressEvent(ev)

    # ══════════════════════════════════════════════════════════════
    #  TIMERS
    # ══════════════════════════════════════════════════════════════
    def _start_timers(self):
        self._ui_tmr = QTimer(self)
        self._ui_tmr.timeout.connect(self._tick_ui)
        self._ui_tmr.start(50)

        self._clk = QTimer(self)
        self._clk.timeout.connect(
            lambda: self.time_lbl.setText(time.strftime("%H:%M:%S")))
        self._clk.start(1000)

    def _tick_ui(self):
        self.pfd.update()
        self.compass.update()
        for vb in self.vthr:
            vb.update()
        self.arm_disp.update()

        self.dep_lbl.setText(f"{self.st.depth:.1f} m")
        self.tmp_lbl.setText(f"{self.st.temperature:.1f} °C")
        self.prs_lbl.setText(f"{self.st.depth * 0.0098:.2f} bar")
        self.amp_lbl.setText(f"{self.st.amperage:.1f} A")
        self.amp_bar.setValue(int(min(self.st.amperage / 10 * 100, 100)))
        self.acc_lbl["X"].setText(f"{self.st.accel_x:.2f}")
        self.acc_lbl["Y"].setText(f"{self.st.accel_y:.2f}")
        self.acc_lbl["Z"].setText(f"{self.st.accel_z:.2f}")
        self.dist_lbl.setText(f"{self.st.distance:.1f} cm")

        self.stl_viewer.set_rotation(
            self.st.pitch, self.st.heading, self.st.roll)
        self.stl_rot_lbl.setText(
            f"Pitch {self.st.pitch:.0f}°  "
            f"Yaw {self.st.heading:.0f}°  "
            f"Roll {self.st.roll:.0f}°")

        # ── Override mode display ──
        override = self.st.override_mode
        olabel = OVERRIDE_LABELS.get(override, f"0x{override:02X}")
        if override == 0x00:
            ocol = "#10b981"    # green — everything forced online
            oicon = "🟢"
        elif override == 0xFF:
            ocol = "#ef4444"    # red — everything forced offline
            oicon = "🔴"
        else:
            ocol = "#fbbf24"    # amber — automatic
            oicon = "⚡"
        self.override_lbl.setText(f"{oicon} Override: {olabel}")
        self.override_lbl.setStyleSheet(
            f"color:{ocol};border:none;font-size:10px;font-weight:bold;")

        # ── Throttle display ──
        tp = self.st.throttle_percent
        if tp > 0:
            tcol = "#ef4444" if tp > 50 else "#fbbf24"
            self.throttle_lbl.setText(f"🔋 Throttle Limited: {tp}%")
            self.throttle_lbl.setStyleSheet(
                f"color:{tcol};border:none;font-size:10px;font-weight:bold;")
        else:
            self.throttle_lbl.setText("🔋 Throttle: None")
            self.throttle_lbl.setStyleSheet(
                "color:#10b981;border:none;font-size:10px;")

        # ── Shutdown bitmask display ──
        sb = self.st.shutdown_bitmask
        if sb:
            shut = [SHUTDOWN_NAMES[i] for i in range(len(SHUTDOWN_NAMES))
                    if sb & (1 << i)]
            self.shutdown_lbl.setText(f"🔌 Offline: {', '.join(shut)}")
            self.shutdown_lbl.setStyleSheet(
                "color:#ef4444;border:none;font-size:10px;font-weight:bold;")
        else:
            self.shutdown_lbl.setText("🔌 Modules: All Online")
            self.shutdown_lbl.setStyleSheet(
                "color:#10b981;border:none;font-size:10px;")

        # ── Active special modes ──
        modes = []
        if self.st.autostab:
            modes.append("✈ Autostab")
        if self.st.false_swipe:
            modes.append("🎯 Precision")
        if self.st.lights:
            modes.append("💡 Lights")
        if self.st.emergency:
            modes.append("⛔ E-STOP")
        if modes:
            self.modes_lbl.setText("  ".join(modes))
            self.modes_lbl.setStyleSheet(
                "color:#60a5fa;border:none;font-size:10px;font-weight:bold;")
        else:
            self.modes_lbl.setText("No special modes active")
            self.modes_lbl.setStyleSheet(
                "color:#6b7280;border:none;font-size:10px;")

        # ── Alerts ──
        a = []
        if self.st.amperage > 25:
            a.append("⚠ High current, auto-limiting and rerouting power")
        if self.st.temperature > 40:
            a.append("Temperature module calibration error")
        if self.st.depth > 400:
            a.append("Pressure module calibration error")
        thruster_inop = self.st.thruster_inop
        if isinstance(thruster_inop, list):
            inop = [i for i in range(8) if thruster_inop[i]]
            if inop:
                a.append(
                    f"🔧 T{','.join(str(j+1) for j in inop)} INOP")
        if self.st.throttle_percent > 80:
            a.append("⚠ Thruster throttle >80% — check load")
        if self.st.override_mode == 0xFF:
            a.append("⚠ Manual power cut active — all modules offline")
        if a:
            self.alert_lbl.setText("\n".join(a))
            self.alert_lbl.setStyleSheet(
                "color:#ef4444;border:none;font-size:11px;")
        else:
            self.alert_lbl.setText("No alerts")
            self.alert_lbl.setStyleSheet(
                "color:#10b981;border:none;font-size:11px;")

        # sync setup checkboxes with incoming state (if core changed them)
        tp_list = self.st.thruster_powered
        if isinstance(tp_list, list):
            for i, cb in enumerate(self.s_pwr):
                if i < len(tp_list):
                    cb.blockSignals(True)
                    cb.setChecked(tp_list[i])
                    cb.blockSignals(False)

    # ══════════════════════════════════════════════════════════════
    #  ACTIONS
    # ══════════════════════════════════════════════════════════════
    def _estop(self):
        self.cmd.send(action="estop")
        for cb in self.s_pwr:
            cb.blockSignals(True)
            cb.setChecked(False)
            cb.blockSignals(False)

    def _meas_start(self):
        self.cmd.send(action="measure_start")

    def _meas_stop(self):
        self.cmd.send(action="measure_stop")

    def _meas_reset(self):
        self.cmd.send(action="measure_reset")

    # ══════════════════════════════════════════════════════════════
    #  CLEANUP
    # ══════════════════════════════════════════════════════════════
    def closeEvent(self, ev):
        self.state_sub.stop()
        self.cam_sub.stop()
        self.cmd.close()
        self.state_sub.wait(2000)
        self.cam_sub.wait(2000)
        super().closeEvent(ev)


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())