import math
import struct
import utime
from machine import I2C, Pin


# ---------- Tunable constants ----------
# DLPF options (MPU6050 register 0x1A):
#   0x03 ≈ 44 Hz (previous default)
#   0x04 ≈ 21 Hz
#   0x05 ≈ 10 Hz
#   0x06 ≈ 5  Hz
DLPF_CFG = 0x04  # lower bandwidth to better reject grinding vibration

# Complementary filter gains
ACC_ANGLE_LP_TAU_S = 0.75       # accel angle low-pass time constant (seconds)
CORRECTION_RATE_HZ = 0.65       # how quickly to pull gyro angle toward accel when steady (1/s)

# Vibration detection on gyro magnitude (deg/s)
VIB_RMS_TAU_S = 0.25            # RMS smoothing window (seconds)
VIB_LOW_THRESHOLD = 25.0        # below this, accel is trusted fully
VIB_HIGH_THRESHOLD = 120.0      # above this, accel corrections are almost disabled

# ---------- Minimal MPU6050 driver ----------
class MPU6050:
    def __init__(self, i2c_obj, addr=0x68, dlpf_cfg=DLPF_CFG):
        self.i2c = i2c_obj
        self.addr = addr
        self.dlpf_cfg = dlpf_cfg
        self._init_device()

    def _init_device(self):
        try:
            # Wake up device
            self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
            utime.sleep_ms(100)
            # ±2g accel config
            self.i2c.writeto_mem(self.addr, 0x1C, b'\x00')
            utime.sleep_ms(10)
            # Set DLPF (see DLPF_CFG constants above)
            self.i2c.writeto_mem(self.addr, 0x1A, bytes([self.dlpf_cfg & 0x07]))
            utime.sleep_ms(10)
        except OSError:
            pass

    def get_accel_gyro(self):
        data = self.i2c.readfrom_mem(self.addr, 0x3B, 14)
        ax, ay, az, _temp, gx, gy, gz = struct.unpack('>hhhhhhh', data)
        accel_scale = 16384.0  # ±2g
        gyro_scale = 131.0     # ±250 dps
        return (
            (ax / accel_scale, ay / accel_scale, az / accel_scale),
            (gx / gyro_scale, gy / gyro_scale, gz / gyro_scale),
        )


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
        dlpf_cfg: int = DLPF_CFG,
    ):
        self.i2c = i2c
        self.mpu = MPU6050(i2c, addr=mpu_addr, dlpf_cfg=dlpf_cfg)
        self.calibration_delay_ms = calibration_delay_ms
        self._set_axis(angle_mode)
        self.g_ref = (0.0, 0.0, 1.0)
        self._last_delta = 0.0
        self._last_good_accel = (0.0, 0.0, 1.0)
        self._last_good_gyro = (0.0, 0.0, 0.0)
        self._last_read_ms = utime.ticks_ms()
        self._angle = 0.0
        self._acc_angle_lp = 0.0
        self._vib_rms_sq = 0.0
        self._gyro_bias = (0.0, 0.0, 0.0)

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
        reading = _safe_read(self.mpu.get_accel_gyro)
        if reading is None:
            return False
        g, gyro = reading
        self._last_read_ms = utime.ticks_ms()
        self.g_ref = _vec_norm(g)
        self._last_good_accel = g
        self._last_good_gyro = gyro
        # Capture gyro bias at rest so integration starts from zero-rate
        self._gyro_bias = gyro
        acc_angle0 = self._calc_acc_angle(g)
        self._last_delta = 0.0
        self._angle = acc_angle0
        self._acc_angle_lp = acc_angle0
        self._vib_rms_sq = 0.0
        return True

    def _calc_acc_angle(self, accel):
        return _signed_angle_about_axis(self.g_ref, accel, self.axis)

    def _gyro_rate_about_axis(self, gyro):
        gx = gyro[0] - self._gyro_bias[0]
        gy = gyro[1] - self._gyro_bias[1]
        gz = gyro[2] - self._gyro_bias[2]
        return _vec_dot((gx, gy, gz), self.axis)

    def _update_filters(self, dt_s, gyro, acc_angle):
        # Low-pass accel-derived angle
        alpha_acc = 1.0 - math.exp(-dt_s / ACC_ANGLE_LP_TAU_S) if dt_s > 0 else 0.0
        self._acc_angle_lp += alpha_acc * (acc_angle - self._acc_angle_lp)

        # Vibration level from gyro RMS magnitude
        gyro_mag = math.sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2])
        alpha_vib = 1.0 - math.exp(-dt_s / VIB_RMS_TAU_S) if dt_s > 0 else 0.0
        self._vib_rms_sq += alpha_vib * (gyro_mag * gyro_mag - self._vib_rms_sq)

    def _vibration_correction_weight(self):
        vib = math.sqrt(self._vib_rms_sq) if self._vib_rms_sq > 0 else 0.0
        if vib <= VIB_LOW_THRESHOLD:
            return 1.0
        if vib >= VIB_HIGH_THRESHOLD:
            return 0.05
        span = VIB_HIGH_THRESHOLD - VIB_LOW_THRESHOLD
        scale = 1.0 - (vib - VIB_LOW_THRESHOLD) / span
        return 0.05 + 0.95 * max(0.0, min(1.0, scale))

    def _fuse(self, dt_s, gyro_rate, acc_angle):
        # Integrate gyro
        self._angle += gyro_rate * dt_s
        # Pull toward accel based on vibration-aware gain
        weight = self._vibration_correction_weight()
        gain = min(1.0, max(0.0, CORRECTION_RATE_HZ * dt_s * weight))
        self._angle += gain * (self._acc_angle_lp - self._angle)

    def get_delta(self):
        reading = _safe_read(self.mpu.get_accel_gyro)
        prev_ts = self._last_read_ms
        if reading is None:
            accel = self._last_good_accel
            gyro = self._last_good_gyro
            ts = self._last_read_ms
        else:
            accel, gyro = reading
            ts = utime.ticks_ms()
            self._last_good_accel = accel
            self._last_good_gyro = gyro
            self._last_read_ms = ts

        if prev_ts is None or ts is None:
            dt_s = 0.0
        else:
            dt_ms = utime.ticks_diff(ts, prev_ts)
            dt_s = max(0.0, dt_ms / 1000.0)

        acc_angle = self._calc_acc_angle(accel)
        gyro_rate = self._gyro_rate_about_axis(gyro)

        self._update_filters(dt_s, gyro, acc_angle)
        self._fuse(dt_s, gyro_rate, acc_angle)

        self._last_delta = self._angle
        return self._angle

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
    dlpf_cfg=DLPF_CFG,
):
    i2c = I2C(i2c_id, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=freq_hz)
    return AngleTracker(
        i2c,
        angle_mode=angle_mode,
        mpu_addr=mpu_addr,
        calibration_delay_ms=calibration_delay_ms,
        dlpf_cfg=dlpf_cfg,
    )
