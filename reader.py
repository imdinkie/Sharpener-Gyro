import math
import struct
import utime
from machine import I2C, Pin

# ---------- Minimal MPU6050 driver ----------
class MPU6050:
    def __init__(self, i2c_obj, addr=0x68):
        self.i2c = i2c_obj
        self.addr = addr
        self._init_device()

    def _init_device(self):
        try:
            # Wake up device
            self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
            utime.sleep_ms(100)
            # ±2g accel config
            self.i2c.writeto_mem(self.addr, 0x1C, b'\x00')
            utime.sleep_ms(10)
            # Enable DLPF (~44 Hz) to reduce vibration noise while staying responsive
            self.i2c.writeto_mem(self.addr, 0x1A, b'\x03')  # ~44 Hz cutoff
            utime.sleep_ms(10)
        except OSError:
            pass

    def get_accel_data(self):
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 6)
        ax, ay, az = struct.unpack('>hhh', data)
        scale = 16384.0
        return (ax / scale, ay / scale, az / scale)


# ---------- Vector helpers ----------
def _vec_norm(v):
    x, y, z = v
    m = math.sqrt(x*x + y*y + z*z)
    if m == 0:
        return (0.0, 0.0, 0.0)
    return (x/m, y/m, z/m)

def _vec_dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def _vec_cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])

def _signed_angle_about_axis(v0, v1, axis_unit):
    v0u = _vec_norm(v0)
    v1u = _vec_norm(v1)
    cross = _vec_cross(v0u, v1u)
    s = _vec_dot(axis_unit, cross)
    c = _vec_dot(v0u, v1u)
    ang = math.degrees(math.atan2(s, c))
    return -180.0 if ang == 180.0 else ang


def _safe_read(read_fn, retries=3, delay_ms=5):
    for _ in range(retries):
        try:
            return read_fn()
        except OSError:
            utime.sleep_ms(delay_ms)
    print("[sensor] read failed after", retries, "retries")
    return None


# ---------- Public API ----------
class AngleTracker:
    """
    Tracks relative angle change of the gravity vector about a fixed axis.
    Modes:
      - "PITCH": about +Y axis
      - "ROLL" : about +X axis
      - "YAW"  : about +Z axis
    """
    def __init__(
        self,
        i2c: I2C,
        *,
        angle_mode: str = "PITCH",
        mpu_addr: int = 0x68,
        calibration_delay_ms: int = 2000,
    ):
        self.i2c = i2c
        self.mpu = MPU6050(i2c, addr=mpu_addr)
        self.calibration_delay_ms = calibration_delay_ms
        self._set_axis(angle_mode)
        self.g_ref = (0.0, 0.0, 1.0)
        self._last_delta = 0.0
        self._last_good = (0.0, 0.0, 1.0)
        self._last_read_ms = utime.ticks_ms()

    def _set_axis(self, angle_mode: str):
        mode = (angle_mode or "PITCH").upper()
        if mode == "ROLL":
            self.axis = _vec_norm((1.0, 0.0, 0.0))
            self.angle_mode = "ROLL"
        elif mode == "YAW":
            self.axis = _vec_norm((0.0, 0.0, 1.0))
            self.angle_mode = "YAW"
        else:
            self.axis = _vec_norm((0.0, 1.0, 0.0))
            self.angle_mode = "PITCH"

    def recalibrate(self) -> bool:
        utime.sleep_ms(self.calibration_delay_ms)
        g = _safe_read(self.mpu.get_accel_data)
        if g is None:
            return False
        self._last_read_ms = utime.ticks_ms()
        self.g_ref = _vec_norm(g)
        self._last_good = g
        self._last_delta = 0.0
        return True

    def get_delta(self):
        g_now = _safe_read(self.mpu.get_accel_data)
        if g_now is None:
            g_now = self._last_good
            ts = self._last_read_ms
        else:
            ts = utime.ticks_ms()
            self._last_good = g_now
            self._last_read_ms = ts
        d = _signed_angle_about_axis(self.g_ref, g_now, self.axis)
        self._last_delta = d
        return d

    def get_last_delta(self):
        return self._last_delta

    def get_last_age_ms(self):
        if self._last_read_ms is None:
            return None
        return utime.ticks_diff(utime.ticks_ms(), self._last_read_ms)

    def get_measurement(self):
        delta = self.get_delta()
        age_ms = self.get_last_age_ms()
        return delta, age_ms

    def set_angle_mode(self, angle_mode: str):
        self._set_axis(angle_mode)


# ---------- Convenience creator ----------
def create_default_tracker(
    *,
    angle_mode="PITCH",
    i2c_id=0,
    scl_pin=22,
    sda_pin=21,
    freq_hz=400_000,
    mpu_addr=0x68,
    calibration_delay_ms=2000,
):
    i2c = I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq_hz)
    return AngleTracker(
        i2c,
        angle_mode=angle_mode,
        mpu_addr=mpu_addr,
        calibration_delay_ms=calibration_delay_ms,
    )
