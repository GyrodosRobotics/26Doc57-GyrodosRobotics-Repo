#!/usr/bin/env python3
"""
26Cod71a
LUCARIO Core [ver 34] v2.2 — headless ROV control backend

Manages:
  • Serial communication with Arduino Mega over RS422
  • Nintendo Switch Pro Controller input via pygame
  • 6-DOF thruster allocation via precomputed mixing matrix
  • Camera feeds over ZMQ
  • State publishing and command handling over ZMQ
  • Power management override relay (v2.2)

Gyrodos Robotics

TO solve 5556 in use issue:
lsof -i:5556
kill -9 <PID>
"""

import os
import sys
import zmq
import json
import time
import struct
import threading
import math
import argparse
import numpy as np
from dataclasses import dataclass, field, asdict
from typing import List, Optional

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import serial as pyserial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import pygame
    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False


# ═══════════════════════════════════════════════════════════════
#  ROV STATE (single source of truth)
# ═══════════════════════════════════════════════════════════════
@dataclass
class ROVState:
    # Attitude & navigation
    pitch: float = 0.0              # degrees (from accel, positive = nose up)
    roll: float = 0.0               # degrees (from accel, positive = right down)
    heading: float = 0.0            # degrees (requires magnetometer — placeholder)
    depth: float = 0.0              # metres

    # Derived motion (placeholder — integrate later)
    speed: float = 0.0
    vert_speed: float = 0.0

    # Environment
    temperature: float = 0.0        # °C
    pressure: float = 0.0           # mbar
    amperage: float = 0.0           # A

    # Thrusters — display values in -100..+100  (0 = neutral)
    thruster_val: List[int] = field(default_factory=lambda: [0] * 8)
    thruster_powered: List[bool] = field(default_factory=lambda: [True] * 8)
    thruster_inverted: List[bool] = field(default_factory=lambda: [False] * 8)
    thruster_inop: List[bool] = field(default_factory=lambda: [False] * 8)
    thruster_trim: List[float] = field(default_factory=lambda: [0.0] * 8)

    # Manipulators
    left_stepper_deg: float = 0.0
    right_stepper_deg: float = 0.0
    left_servo_deg: float = 90.0
    right_servo_deg: float = 90.0

    # Raw IMU
    accel_x: float = 0.0           # g
    accel_y: float = 0.0           # g
    accel_z: float = 0.0           # g
    gyro_x: float = 0.0            # °/s
    gyro_y: float = 0.0            # °/s
    gyro_z: float = 0.0            # °/s

    # Tool / measurement
    distance: float = 0.0
    measuring: bool = False

    # Modes & flags
    autostab: bool = False
    false_swipe: bool = False
    lights: bool = False
    emergency: bool = False
    right_claw_open: bool = False
    left_claw_open: bool = False
    serial_connected: bool = False
    controller_connected: bool = False

    # Power management (v2.2 — from Arduino v1.2 telemetry)
    throttle_percent: int = 0       # 0–100, auto-throttle on thrusters
    shutdown_bitmask: int = 0       # 8-bit module shutdown status
    override_mode: int = 0x88       # 0x00=forbid, 0x88=auto, 0xFF=manual cut
    controls_locked: bool = False   # v2.2 — lock controller input from GUI



# ═══════════════════════════════════════════════════════════════
#  THRUSTER MIXING MATRIX
# ═══════════════════════════════════════════════════════════════
class ThrusterController:
    """
    6-DOF wrench (fx, fy, fz, tx, ty, tz) → 8 thruster powers
    via precomputed pseudo-inverse of the geometry matrix.
    """

    def __init__(self):
        positions = np.array([
            [ 24,  16,  8], [ 24, -16,  8],
            [-24,  16,  8], [-24, -16,  8],
            [ 24,  16, -8], [ 24, -16, -8],
            [-24,  16, -8], [-24, -16, -8],
        ], dtype=np.float32)

        directions = np.array([
            [ 1, -1, -1], [ 1,  1, -1],
            [-1, -1, -1], [-1,  1, -1],
            [ 1, -1,  1], [ 1,  1,  1],
            [-1, -1,  1], [-1,  1,  1],
        ], dtype=np.float32)
        directions /= np.linalg.norm(directions[0])  # normalise

        # Build 6×8 mixing matrix  [forces; torques] = M · thrusters
        M = np.zeros((6, 8), dtype=np.float32)
        for i in range(8):
            r, f = positions[i], directions[i]
            M[:3, i] = f                                    # force
            M[3, i] = r[1] * f[2] - r[2] * f[1]            # roll  torque
            M[4, i] = r[2] * f[0] - r[0] * f[2]            # pitch torque
            M[5, i] = r[0] * f[1] - r[1] * f[0]            # yaw   torque

        self._M_inv = np.linalg.pinv(M).astype(np.float32)  # 8×6

    def allocate(self, fx: float, fy: float, fz: float,
                 tx: float, ty: float, tz: float) -> np.ndarray:
        """Returns 8 thruster powers in [-1.0, +1.0]."""
        wrench = np.array([fx, fy, fz, tx, ty, tz], dtype=np.float32)
        powers = self._M_inv @ wrench

        peak = np.abs(powers).max()
        if peak < 1e-6:
            return np.zeros(8, dtype=np.float32)

        powers /= peak

        command_mag = min(float(np.abs(wrench).max()), 1.0)
        powers *= command_mag

        return powers


# ═══════════════════════════════════════════════════════════════
#  COMPLEMENTARY FILTER — fuse accel + gyro for stable attitude
# ═══════════════════════════════════════════════════════════════
class ComplementaryFilter:
    """
    Blends gyroscope (fast, drifts) with accelerometer (slow, noisy).

    alpha close to 1.0 → trust gyro more (smooth, but drifts over time)
    alpha close to 0.0 → trust accel more (noisy, but no drift)

    0.96–0.98 is typical for 50 Hz update with moderate vibration.
    """

    def __init__(self, alpha: float = 0.97):
        self.alpha = alpha
        self.pitch = 0.0
        self.roll = 0.0
        self._initialized = False

    def update(self, accel_pitch: float, accel_roll: float,
               gyro_x: float, gyro_y: float, dt: float):
        if not self._initialized:
            self.pitch = accel_pitch
            self.roll = accel_roll
            self._initialized = True
            return

        self.pitch = self.alpha * (self.pitch + gyro_y * dt) \
                     + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * (self.roll + gyro_x * dt) \
                    + (1 - self.alpha) * accel_roll


# ═══════════════════════════════════════════════════════════════
#  PID CONTROLLER
# ═══════════════════════════════════════════════════════════════
class PIDController:

    def __init__(self, kp: float, ki: float, kd: float,
                 integral_limit: float = 0.3,
                 output_limit: float = 0.4,
                 derivative_filter: float = 0.7):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.d_filter = derivative_filter

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._initialized = False

    def update(self, target: float, measured: float, dt: float) -> float:
        error = target - measured

        p = self.kp * error

        self._integral += error * dt
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        if not self._initialized:
            raw_d = 0.0
            self._initialized = True
        else:
            raw_d = (error - self._prev_error) / dt

        filtered_d = (self.d_filter * self._prev_derivative
                      + (1 - self.d_filter) * raw_d)
        self._prev_derivative = filtered_d
        d = self.kd * filtered_d

        self._prev_error = error

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._initialized = False

# ═══════════════════════════════════════════════════════════════
#  SERIAL INTERFACE — RS422  ↔  Arduino Mega
# ═══════════════════════════════════════════════════════════════
#
#  Packet wire format (both directions):
#    [0xAA] [CMD] [LEN] [PAYLOAD × LEN] [CHECKSUM]
#    checksum = (sum of all preceding bytes) & 0xFF
#
#  TX  CMD 0x01  Thruster   LEN=8   payload: 8 × uint8  (0–200, 100=stop)
#  TX  CMD 0x02  Actuator   LEN=8   payload: see spec
#    [7] override  0x00=forbid shutdown, 0x88=auto, 0xFF=manual cut
#  RX  CMD 0x03  Telemetry  LEN=20  payload:
#    [0-13]  6 × int16 + 1 × uint16  (LE) — IMU + depth
#    [14]    throttle_percent  uint8  0–100
#    [15]    shutdown_bitmask  uint8  (bit per module)
#    [16-17] temperature       int16  centi-°C
#    [18-19] amperage          uint16 milli-A
#
# ═══════════════════════════════════════════════════════════════

class SerialInterface:

    START   = 0xAA
    CMD_THR = 0x01
    CMD_ACT = 0x02
    CMD_TEL = 0x03

    # parser states
    S_START, S_CMD, S_LEN, S_DATA, S_CS = range(5)

    def __init__(self, state: ROVState,
                 port: str = '/dev/ttyUSB0', baud: int = 115200):
        self.st = state
        self._port = port
        self._baud = baud
        self._ser: Optional[pyserial.Serial] = None
        self._lock = threading.Lock()       # guards _ser.write()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_rx: float = 0.0          # monotonic timestamp
        # parser scratch
        self._ps   = self.S_START
        self._cmd  = 0
        self._dlen = 0
        self._buf  = bytearray()
        self._idx  = 0
        self._csum = 0


    # ── lifecycle ──

    def connect(self) -> bool:
        if not HAS_SERIAL:
            print("[SERIAL] pyserial not installed — running without hardware")
            return False
        try:
            self._ser = pyserial.Serial(self._port, self._baud, timeout=0.01)
            time.sleep(0.1)
            self._running = True
            self._thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._thread.start()
            self.st.serial_connected = True
            print(f"[SERIAL] Opened {self._port} @ {self._baud}")
            return True
        except Exception as e:
            print(f"[SERIAL] Connect failed: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._ser and self._ser.is_open:
            self._ser.close()
        self.st.serial_connected = False
        print("[SERIAL] Closed")

    # ── transmit helpers ──

    def _packet(self, cmd: int, payload: bytes) -> bytes:
        hdr = bytes([self.START, cmd, len(payload)])
        cs = (sum(hdr) + sum(payload)) & 0xFF
        return hdr + payload + bytes([cs])

    def send_thruster(self, values: List[int]):
        """Send CMD 0x01.  values: list of 8 ints, 0–200, 100 = neutral."""
        if not self._ser or not self._ser.is_open:
            return
        data = bytes(max(0, min(200, v)) for v in values)
        with self._lock:
            self._ser.write(self._packet(self.CMD_THR, data))

    def send_actuator(self, servo_r: int, servo_l: int,
                      claw_r: int, claw_l: int,
                      cont: int, lights: int,
                      status: int, override: int = 0x88):
        """Send CMD 0x02.  All values are single bytes, clamped to spec.
        override: 0x00=forbid shutdown, 0x88=auto, 0xFF=manual cut."""
        if not self._ser or not self._ser.is_open:
            return
        data = bytes([
            max(0, min(180, servo_r)),
            max(0, min(180, servo_l)),
            max(0, min(200, claw_r)),
            max(0, min(200, claw_l)),
            max(0, min(200, cont)),
            max(0, min(100, lights)),
            status & 0xFF,
            override & 0xFF,
        ])
        with self._lock:
            self._ser.write(self._packet(self.CMD_ACT, data))

    # ── receive (background thread) ──

    def _rx_loop(self):
        while self._running:
            try:
                if self._ser and self._ser.in_waiting:
                    for b in self._ser.read(self._ser.in_waiting):
                        self._feed(b)
                else:
                    time.sleep(0.001)
            except Exception as e:
                print(f"[SERIAL RX] {e}")
                time.sleep(0.1)

    def _feed(self, b: int):
        """Byte-by-byte packet parser — mirrors the Arduino-side parser."""
        s = self._ps
        if s == self.S_START:
            if b == self.START:
                self._csum = b
                self._ps = self.S_CMD
        elif s == self.S_CMD:
            self._cmd = b
            self._csum += b
            self._ps = self.S_LEN
        elif s == self.S_LEN:
            self._dlen = b
            self._csum += b
            if 0 < b <= 32:
                self._buf = bytearray(b)
                self._idx = 0
                self._ps = self.S_DATA
            else:
                self._ps = self.S_START
        elif s == self.S_DATA:
            self._buf[self._idx] = b
            self._idx += 1
            self._csum += b
            if self._idx >= self._dlen:
                self._ps = self.S_CS
        elif s == self.S_CS:
            if (self._csum & 0xFF) == b:
                self._dispatch()
            self._ps = self.S_START

    def _dispatch(self):
        if self._cmd == self.CMD_TEL and self._dlen == 20:
            self._parse_telemetry()

    def _parse_telemetry(self):
        """
        CMD 0x03 — 20-byte payload (little-endian):
          [0-1]   accel X    int16   milli-g
          [2-3]   accel Y    int16   milli-g
          [4-5]   accel Z    int16   milli-g
          [6-7]   gyro  X    int16   deci-°/s
          [8-9]   gyro  Y    int16   deci-°/s
          [10-11] gyro  Z    int16   deci-°/s
          [12-13] depth      uint16  mm
          [14]    throttle % uint8   0–100
          [15]    shutdown   uint8   bitmask
          [16-17] temperature int16  centi-°C
          [18-19] amperage   uint16  milli-A
        """
        ax, ay, az, gx, gy, gz, dmm = struct.unpack_from('<hhhhhhH', self._buf)

        self.st.accel_x = ax / 1000.0
        self.st.accel_y = ay / 1000.0
        self.st.accel_z = az / 1000.0
        self.st.gyro_x = gx / 10.0
        self.st.gyro_y = gy / 10.0
        self.st.gyro_z = gz / 10.0
        self.st.depth = dmm / 1000.0

        self.st.throttle_percent = min(self._buf[14], 100)
        self.st.shutdown_bitmask = self._buf[15]

        temp_centi, = struct.unpack_from('<h', self._buf, 16)
        self.st.temperature = temp_centi / 100.0

        amps_milli, = struct.unpack_from('<H', self._buf, 18)
        self.st.amperage = amps_milli / 1000.0

        az_safe = max(abs(self.st.accel_z), 0.01)
        self.st.pitch = math.degrees(math.atan2(
            self.st.accel_x,
            math.sqrt(self.st.accel_y ** 2 + az_safe ** 2)))
        self.st.roll = math.degrees(math.atan2(
            self.st.accel_y,
            math.sqrt(self.st.accel_x ** 2 + az_safe ** 2)))

        self._last_rx = time.monotonic()

    @property
    def telemetry_age(self) -> float:
        """Seconds since last good telemetry packet (inf if none yet)."""
        if self._last_rx == 0.0:
            return float('inf')
        return time.monotonic() - self._last_rx


# ═══════════════════════════════════════════════════════════════
#  NINTENDO SWITCH PRO CONTROLLER
# ═══════════════════════════════════════════════════════════════

class ControllerInput:
    """Nintendo Switch Pro Controller via pygame — polling + edge detect."""

    A, B, X, Y          = 0, 1, 2, 3
    MINUS, HOME, PLUS   = 4, 5, 6
    LSTICK, RSTICK      = 7, 8
    L, R                = 9, 10
    DUP, DDOWN          = 11, 12
    DLEFT, DRIGHT       = 13, 14
    CAPTURE             = 15

    DEADZONE = 0.12

    def __init__(self):
        self._joy = None
        self.axes = [0.0] * 6
        self._center = [0.0] * 6
        self.buttons = [False] * 16
        self._prev = [False] * 16
        self.connected = False

    def init(self) -> bool:
        if not HAS_PYGAME:
            print("[CTRL] pygame not installed — no controller input")
            return False

        pygame.init()

        count = pygame.joystick.get_count()
        if count == 0:
            print("[CTRL] No controller detected")
            return False

        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()
        self.connected = True
        print(f"[CTRL] {self._joy.get_name()} — "
              f"{self._joy.get_numaxes()} axes, "
              f"{self._joy.get_numbuttons()} buttons")

        time.sleep(0.1)
        self.recalibrate_center()

        return True

    def recalibrate_center(self):
        if not self.connected:
            return
        pygame.event.pump()
        n_axes = min(6, self._joy.get_numaxes())
        self._center = [self._joy.get_axis(i) for i in range(n_axes)]
        while len(self._center) < 6:
            self._center.append(0.0)
        print(f"[CTRL] Center calibrated: "
              f"{[f'{c:+.4f}' for c in self._center]}")

    def poll(self):
        if not self.connected:
            return
        pygame.event.pump()
        self._prev = self.buttons[:]
        for i in range(min(6, self._joy.get_numaxes())):
            v = self._joy.get_axis(i) - self._center[i]
            self.axes[i] = v if abs(v) > self.DEADZONE else 0.0
        for i in range(min(16, self._joy.get_numbuttons())):
            self.buttons[i] = bool(self._joy.get_button(i))

    def pressed(self, btn: int) -> bool:
        return self.buttons[btn] and not self._prev[btn]

    def held(self, btn: int) -> bool:
        return self.buttons[btn]

    @property
    def left_x(self):  return self.axes[0]
    @property
    def left_y(self):  return self.axes[1]
    @property
    def right_x(self): return self.axes[2]
    @property
    def right_y(self): return self.axes[3]

    def shutdown(self):
        if self._joy:
            self._joy.quit()
        if HAS_PYGAME:
            pygame.quit()


# ═══════════════════════════════════════════════════════════════
#  CAMERA MANAGER
# ═══════════════════════════════════════════════════════════════
class CameraManager:

    def __init__(self, ctx: zmq.Context, port: int = 5556):
        self._caps: dict = {}
        self._pub = ctx.socket(zmq.PUB)
        try:
            self._pub.bind(f"tcp://*:{port}")
        except zmq.error.ZMQError:
            print(f"[CAM] Port {port} still held — forcing rebind...")
            self._pub.close()
            self._pub = ctx.socket(zmq.PUB)
            self._pub.setsockopt(zmq.LINGER, 0)
            self._pub.setsockopt(zmq.CONFLATE, 1)
            self._pub.bind(f"tcp://*:{port}")
        self._active: set = set()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def request_camera(self, idx: int) -> bool:
        if idx in self._caps:
            self._active.add(idx)
            return True
        if not HAS_CV2:
            return False
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            self._caps[idx] = cap
            self._active.add(idx)
            return True
        return False

    def release_camera(self, idx: int):
        self._active.discard(idx)
        cap = self._caps.pop(idx, None)
        if cap is not None:
            cap.release()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        for c in self._caps.values():
            c.release()
        self._caps.clear()

    def _loop(self):
        while self._running:
            for idx in list(self._active):
                cap = self._caps.get(idx)
                if cap and cap.isOpened():
                    ok, frame = cap.read()
                    if ok:
                        _, jpg = cv2.imencode(
                            '.jpg', frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 80])
                        self._pub.send_multipart([
                            f"cam:{idx}".encode(), jpg.tobytes()])
            time.sleep(0.033)


# ═══════════════════════════════════════════════════════════════
#  COMMAND HANDLER (ZMQ commands from GUI)
# ═══════════════════════════════════════════════════════════════
class CommandHandler:

    def __init__(self, state: ROVState, cam: CameraManager):
        self.st = state
        self.cam = cam

    def handle(self, cmd: dict):
        action = cmd.get("action")

        if action == "estop":
            self.st.emergency = True
            for i in range(8):
                self.st.thruster_powered[i] = False
            print("[CMD] E-STOP ACTIVATED")

        elif action == "estop_release":
            self.st.emergency = False
            for i in range(8):
                self.st.thruster_powered[i] = True
            print("[CMD] E-STOP released")

        elif action == "set_thruster_power":
            idx = cmd.get("idx")
            if isinstance(idx, int) and 0 <= idx < 8:
                self.st.thruster_powered[idx] = cmd["value"]
            else:
                print(f"[CMD] Invalid thruster index: {idx}")

        elif action == "set_thruster_invert":
            self.st.thruster_inverted[cmd["idx"]] = cmd["value"]

        elif action == "set_thruster_trim":
            self.st.thruster_trim[cmd["idx"]] = float(cmd["value"])

        elif action == "set_thruster_inop":
            self.st.thruster_inop[cmd["idx"]] = cmd["value"]

        elif action == "open_camera":
            self.cam.request_camera(cmd["idx"])

        elif action == "release_camera":
            self.cam.release_camera(cmd["idx"])

        elif action == "measure_start":
            self.st.measuring = True

        elif action == "measure_stop":
            self.st.measuring = False

        elif action == "measure_reset":
            self.st.distance = 0.0
            self.st.measuring = False

        elif action == "set_autostab":
            self.st.autostab = cmd.get("value", False)
            print(f"[CMD] Autostab {'ON' if self.st.autostab else 'OFF'}")

        elif action == "set_lights":
            self.st.lights = cmd.get("value", False)

        elif action == "set_mode":
            mode = cmd.get("mode")
            val = cmd.get("value", False)
            if mode == "autostab":
                self.st.autostab = val
                print(f"[CMD] Autostab {'ON' if val else 'OFF'}")
            elif mode == "false_swipe":
                self.st.false_swipe = val
                print(f"[CMD] False Swipe {'ON' if val else 'OFF'}")
            elif mode == "lock":
                self.st.controls_locked = val
                print(f"[CMD] Controls Lock {'ON' if val else 'OFF'}")
            else:
                print(f"[CMD] Unknown mode: {mode}")

        elif action == "set_override":
            val = cmd.get("value", 0x88)
            if val in (0x00, 0x88, 0xFF):
                self.st.override_mode = val
                names = {0x00: "FORBID SHUTDOWN",
                         0x88: "AUTOMATIC",
                         0xFF: "MANUAL CUT"}
                print(f"[CMD] Override → 0x{val:02X} ({names.get(val, '?')})")
            else:
                print(f"[CMD] Invalid override value: 0x{val:02X} — ignored")

        else:
            print(f"[CMD] Unknown action: {action}")


# ═══════════════════════════════════════════════════════════════
#  LUCARIO CORE — MAIN LOOP
# ═══════════════════════════════════════════════════════════════
class LucarioCore:

    STATE_PORT = 5555
    CAM_PORT   = 5556
    CMD_PORT   = 5557

    SERVO_STEP_DEG = 2
    CLAW_RUN_FRAMES = 75
    CLAW_OPEN_SPEED = 200
    CLAW_CLOSE_SPEED = 0
    DPAD_AUTHORITY = 0.5
    FALSE_SWIPE_SCALE = 0.4
    SPOOL_SPEED = 200

    def __init__(self, serial_port: str = '/dev/ttyUSB0',
                 baud: int = 115200):
        self.ctx = zmq.Context()
        self.state = ROVState()

        self.mix     = ThrusterController()
        self.serial  = SerialInterface(self.state, serial_port, baud)
        self.ctrl    = ControllerInput()
        self.cam_mgr = CameraManager(self.ctx, self.CAM_PORT)
        self.cmd_h   = CommandHandler(self.state, self.cam_mgr)

        self._pub = self.ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{self.STATE_PORT}")

        self._sub = self.ctx.socket(zmq.SUB)
        self._sub.bind(f"tcp://*:{self.CMD_PORT}")
        self._sub.setsockopt_string(zmq.SUBSCRIBE, "")

        self._poller = zmq.Poller()
        self._poller.register(self._sub, zmq.POLLIN)

        self._servo_r = 90
        self._servo_l = 90
        self._claw_r_ctr = 0
        self._claw_l_ctr = 0

        self._telem_warned = False

        self._att_filter = ComplementaryFilter(alpha=0.97)
        self._pid_roll = PIDController(kp=0.015, ki=0.002, kd=0.008,
                                       integral_limit=0.3, output_limit=0.4)
        self._pid_pitch = PIDController(kp=0.015, ki=0.002, kd=0.008,
                                        integral_limit=0.3, output_limit=0.4)
        self._pid_depth = PIDController(kp=1.5, ki=0.3, kd=0.8,
                                        integral_limit=0.3, output_limit=0.5)
        self._depth_target = None
        self._autostab_was_on = False

        # Heading (gyro-Z integration — relative, drifts without magnetometer)
        self._heading_raw = 0.0

        # Velocity estimation (accel integration + drag model)
        self._vel_x = 0.0
        self._vel_y = 0.0
        self._vel_z = 0.0
        self._prev_depth = 0.0
        self._vspeed_filtered = 0.0
        self._DRAG_DECAY = 0.7  # 1/s — ~1.4 s time constant in water

        # Distance measurement (position integration)
        self._meas_px = 0.0
        self._meas_py = 0.0
        self._meas_pz = 0.0
        self._meas_was_on = False

        self._last_tick = time.monotonic()


    def run(self):
        print("=" * 52)
        print("  LUCARIO CORE v2.2 — Gyrodos Robotics")
        print(f"  State  → tcp://*:{self.STATE_PORT}")
        print(f"  Camera → tcp://*:{self.CAM_PORT}")
        print(f"  Cmds   ← tcp://*:{self.CMD_PORT}")
        print("=" * 52)

        self.serial.connect()
        self.ctrl.init()
        self.state.controller_connected = self.ctrl.connected
        self.cam_mgr.start()

        try:
            while True:
                self._tick()
                time.sleep(0.02)
        except KeyboardInterrupt:
            print("\n[CORE] Shutting down…")
        finally:
            self._emergency_stop()
            time.sleep(0.05)
            self.serial.disconnect()
            self.ctrl.shutdown()
            self.cam_mgr.stop()
            self.ctx.term()

    def _tick(self):
        st = self.state
        now = time.monotonic()
        self._dt = now - self._last_tick
        self._last_tick = now
        # clamp to avoid huge jumps on lag spikes
        self._dt = max(0.005, min(0.1, self._dt))

        self.ctrl.poll()
        self._update_attitude()
        self._update_velocity()
        self._update_measurement()

        if self.ctrl.connected and not st.emergency:
            self._handle_controller()
        elif st.emergency:
            self._emergency_stop()

        while True:
            evts = dict(self._poller.poll(0))
            if self._sub in evts:
                try:
                    self.cmd_h.handle(json.loads(self._sub.recv_string()))
                except json.JSONDecodeError:
                    pass
            else:
                break

        age = self.serial.telemetry_age
        if age > 2.0 and st.serial_connected and not self._telem_warned:
            print("[CORE] WARNING — no telemetry from Arduino for >2 s")
            self._telem_warned = True
        elif age < 1.0:
            self._telem_warned = False

        self._pub.send_string(json.dumps(asdict(st)))

    def _update_attitude(self):
        st = self.state
        dt = 0.02

        az_safe = max(abs(st.accel_z), 0.01)
        accel_pitch = math.degrees(math.atan2(
            st.accel_x, math.sqrt(st.accel_y ** 2 + az_safe ** 2)))
        accel_roll = math.degrees(math.atan2(
            st.accel_y, math.sqrt(st.accel_x ** 2 + az_safe ** 2)))

        self._att_filter.update(
            accel_pitch, accel_roll,
            st.gyro_x, st.gyro_y,
            dt=dt)

        st.pitch = self._att_filter.pitch
        st.roll = self._att_filter.roll

        # Heading from gyro-Z integration (relative — no magnetometer)
        # Deadzone suppresses drift from gyro bias when stationary
        gz = st.gyro_z
        if abs(gz) < 0.5:
            gz = 0.0
        self._heading_raw += gz * dt
        st.heading = self._heading_raw % 360

    def _update_velocity(self):
        """
        Estimate 3D velocity from accelerometer + hydrodynamic drag model.

        Vertical: depth derivative when depth sensor is active (accurate),
                  accel integration fallback when depth sensor absent.
        Horizontal: gravity-compensated accel integration with drag decay
                    that naturally limits drift — in water, speed decays
                    exponentially without thrust.
        """
        st = self.state
        dt = 0.02

        # ── Remove gravity from accelerometer using attitude ──
        pitch_r = math.radians(st.pitch)
        roll_r = math.radians(st.roll)

        gx_comp = -math.sin(pitch_r)
        gy_comp = math.sin(roll_r) * math.cos(pitch_r)
        gz_comp = math.cos(roll_r) * math.cos(pitch_r)

        lin_x = (st.accel_x - gx_comp) * 9.80665  # g → m/s²
        lin_y = (st.accel_y - gy_comp) * 9.80665
        lin_z = (st.accel_z - gz_comp) * 9.80665

        # ── Integrate acceleration → velocity (horizontal) ──
        self._vel_x += lin_x * dt
        self._vel_y += lin_y * dt

        # ── Hydrodynamic drag decay ──
        drag = 1.0 - self._DRAG_DECAY * dt
        self._vel_x *= drag
        self._vel_y *= drag

        # ── ZUPT: if net linear accel is low, ROV is likely stationary ──
        lin_mag = math.sqrt(lin_x ** 2 + lin_y ** 2 + lin_z ** 2)
        if lin_mag < 0.4:
            self._vel_x *= 0.95
            self._vel_y *= 0.95

        # ── Vertical speed ──
        depth_active = abs(st.depth) > 0.001 or abs(self._prev_depth) > 0.001
        if depth_active:
            # Depth sensor provides accurate vertical measurement
            raw_vs = (st.depth - self._prev_depth) / dt
            self._vspeed_filtered += 0.15 * (raw_vs - self._vspeed_filtered)
            self._vel_z = self._vspeed_filtered
        else:
            # No depth sensor — use accel integration with drag
            self._vel_z += lin_z * dt
            self._vel_z *= drag
            if lin_mag < 0.4:
                self._vel_z *= 0.95
            self._vspeed_filtered = self._vel_z

        self._prev_depth = st.depth

        # ── Publish to state ──
        st.speed = math.sqrt(self._vel_x ** 2 + self._vel_y ** 2)
        st.vert_speed = self._vspeed_filtered * 1000  # mm/s for PFD V/S tape

    def _update_measurement(self):
        """
        Integrate velocity to compute straight-line displacement
        while the measurement tool is active (START → STOP).
        """
        st = self.state
        dt = 0.02

        if st.measuring:
            if not self._meas_was_on:
                self._meas_px = 0.0
                self._meas_py = 0.0
                self._meas_pz = 0.0
                self._meas_was_on = True

            self._meas_px += self._vel_x * dt
            self._meas_py += self._vel_y * dt
            self._meas_pz += self._vel_z * dt

            st.distance = math.sqrt(
                self._meas_px ** 2
                + self._meas_py ** 2
                + self._meas_pz ** 2) * 100.0  # m → cm
        else:
            if self._meas_was_on:
                self._meas_was_on = False

    def _handle_controller(self):
        c = self.ctrl
        st = self.state
        # Always allow recalibrate even when locked
        if c.pressed(c.CAPTURE):
            c.recalibrate_center()

        if st.controls_locked:
            return

        if c.pressed(c.A):
            st.right_claw_open = not st.right_claw_open
            self._claw_r_ctr = self.CLAW_RUN_FRAMES

        if c.pressed(c.B):
            st.left_claw_open = not st.left_claw_open
            self._claw_l_ctr = self.CLAW_RUN_FRAMES

        if c.pressed(c.X):
            st.autostab = not st.autostab
            print(f"[CTRL] Autostab {'ON' if st.autostab else 'OFF'}")

        if c.pressed(c.RSTICK):
            st.false_swipe = not st.false_swipe
            print(f"[CTRL] FALSE SWIPE {'ON' if st.false_swipe else 'OFF'}")

        if c.pressed(c.HOME):
            st.lights = not st.lights

        if c.held(c.PLUS):
            self._servo_r = min(180, self._servo_r + self.SERVO_STEP_DEG)
        if c.held(c.MINUS):
            self._servo_r = max(0, self._servo_r - self.SERVO_STEP_DEG)

        if c.held(c.R):
            self._servo_l = min(180, self._servo_l + self.SERVO_STEP_DEG)
        if c.held(c.L):
            self._servo_l = max(0, self._servo_l - self.SERVO_STEP_DEG)

        st.right_servo_deg = float(self._servo_r)
        st.left_servo_deg = float(self._servo_l)

        fx = -c.left_y
        fy = c.left_x
        fz = -c.right_y
        tz = c.right_x

        tx = 0.0
        ty = 0.0
        if c.held(c.DUP):    ty += self.DPAD_AUTHORITY
        if c.held(c.DDOWN):  ty -= self.DPAD_AUTHORITY
        if c.held(c.DRIGHT): tx += self.DPAD_AUTHORITY
        if c.held(c.DLEFT):  tx -= self.DPAD_AUTHORITY

        if st.autostab:
            if not self._autostab_was_on:
                self._pid_roll.reset()
                self._pid_pitch.reset()
                self._pid_depth.reset()
                self._depth_target = st.depth
                self._autostab_was_on = True

            tx += self._pid_roll.update(target=0.0, measured=st.roll, dt=0.02)
            ty += self._pid_pitch.update(target=0.0, measured=st.pitch, dt=0.02)

            if abs(fz) < 0.05 and self._depth_target is not None:
                fz += self._pid_depth.update(
                    target=self._depth_target, measured=st.depth, dt=0.02)
            else:
                self._depth_target = st.depth
                self._pid_depth.reset()
        else:
            if self._autostab_was_on:
                self._autostab_was_on = False
                self._depth_target = None

        powers = self.mix.allocate(fx, fy, fz, tx, ty, tz)

        if st.false_swipe:
            powers = powers * self.FALSE_SWIPE_SCALE

        thr_wire = []
        for i in range(8):
            p = float(powers[i])

            p += st.thruster_trim[i]
            if st.thruster_inverted[i]:
                p = -p
            if not st.thruster_powered[i] or st.thruster_inop[i]:
                p = 0.0

            p = max(-1.0, min(1.0, p))
            thr_wire.append(int(p * 100 + 100))
            st.thruster_val[i] = int(p * 100)

        self.serial.send_thruster(thr_wire)

        if self._claw_r_ctr > 0:
            claw_r = self.CLAW_OPEN_SPEED if st.right_claw_open else self.CLAW_CLOSE_SPEED
            self._claw_r_ctr -= 1
        else:
            claw_r = 100

        if self._claw_l_ctr > 0:
            claw_l = self.CLAW_OPEN_SPEED if st.left_claw_open else self.CLAW_CLOSE_SPEED
            self._claw_l_ctr -= 1
        else:
            claw_l = 100

        cont = self.SPOOL_SPEED if c.held(c.Y) else 100

        self.serial.send_actuator(
            servo_r=self._servo_r,
            servo_l=self._servo_l,
            claw_r=claw_r,
            claw_l=claw_l,
            cont=cont,
            lights=100 if st.lights else 0,
            status=0xFF,
            override=st.override_mode,
        )

    def _emergency_stop(self):
        self.serial.send_thruster([100] * 8)
        self.serial.send_actuator(
            servo_r=90, servo_l=90,
            claw_r=100, claw_l=100,
            cont=100, lights=0,
            status=0x00,
            override=self.state.override_mode,
        )
        self.state.thruster_val = [0] * 8


# ═══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════
def _detect_serial_port() -> str:
    """Return the first likely USB-serial port for the current OS."""
    import glob, sys

    if sys.platform == 'darwin':
        candidates = glob.glob('/dev/tty.usbserial*') + glob.glob('/dev/cu.usbserial*')
    elif sys.platform == 'linux':
        candidates = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    else:
        candidates = []  # Windows: COM ports handled differently

    if candidates:
        candidates.sort()
        return candidates[0]

    # Fallback per platform
    return '/dev/ttyUSB0' if sys.platform == 'linux' else '/dev/tty.usbserial'


def main():
    default_port = _detect_serial_port()

    ap = argparse.ArgumentParser(
        description="LUCARIO Core [ver 34] — ROV control backend")
    ap.add_argument(
        '--port', default=default_port,
        help=f'Serial port for RS422 adapter (detected: {default_port})')
    ap.add_argument(
        '--baud', type=int, default=115200,
        help='Serial baud rate (default: 115200)')
    args = ap.parse_args()

    core = LucarioCore(serial_port=args.port, baud=args.baud)
    core.run()


if __name__ == "__main__":
    main()
