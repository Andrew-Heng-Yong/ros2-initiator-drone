"""Small gyro-only MPU6050 reader and camera-frame SO(3) integration.

The reader deliberately touches only the MPU6050 gyro output registers while
sampling.  It is safe to construct and start without an attached I2C device;
camera tracking can continue with ``relative_rotation`` returning ``None``.
"""

from __future__ import annotations

from bisect import bisect_left, bisect_right

import fcntl
import os
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


I2C_SLAVE = 0x0703

REG_PWR_MGMT_1 = 0x6B
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_GYRO_OUT = 0x43
REG_WHO_AM_I = 0x75

GYRO_RANGES_DPS = (250, 500, 1000, 2000)
GYRO_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
VALID_WHO_AM_I = (0x68, 0x69)


@dataclass(frozen=True)
class GyroSample:
    """A gyro sample in camera-independent sensor units (rad/s)."""

    timestamp: float
    angular_velocity: np.ndarray


def _timestamp_seconds(value: object) -> float:
    """Convert float-like and ROS-style timestamps to Unix seconds."""

    if hasattr(value, "nanoseconds"):
        value = getattr(value, "nanoseconds") / 1_000_000_000.0
    elif hasattr(value, "sec") and hasattr(value, "nanosec"):
        value = getattr(value, "sec") + getattr(value, "nanosec") / 1_000_000_000.0
    result = float(value)
    if not np.isfinite(result):
        raise ValueError("timestamp must be finite")
    return result


def _vector3(value: Sequence[float] | np.ndarray, name: str = "vector") -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be three finite numbers")
    return result.copy()


def _rotation_matrix(value: Sequence[Sequence[float]] | np.ndarray) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError("rotation_camera_from_gyro must be a finite 3x3 matrix")
    if not np.allclose(result.T @ result, np.eye(3), atol=2e-3, rtol=0.0):
        raise ValueError("rotation_camera_from_gyro must be orthonormal")
    if not np.isclose(np.linalg.det(result), 1.0, atol=2e-3, rtol=0.0):
        raise ValueError("rotation_camera_from_gyro must have determinant +1")
    return result.copy()


def so3_exp(rotation_vector: Sequence[float] | np.ndarray) -> np.ndarray:
    """Return ``exp([rotation_vector]x)`` using Rodrigues' formula."""

    vector = _vector3(rotation_vector, "rotation_vector")
    theta_squared = float(vector @ vector)
    skew = np.array(
        [[0.0, -vector[2], vector[1]],
         [vector[2], 0.0, -vector[0]],
         [-vector[1], vector[0], 0.0]],
        dtype=float,
    )
    if theta_squared < 1.0e-16:
        return np.eye(3) + skew + 0.5 * (skew @ skew)
    theta = float(np.sqrt(theta_squared))
    return np.eye(3) + (np.sin(theta) / theta) * skew + ((1.0 - np.cos(theta)) / theta_squared) * (skew @ skew)


def _sample_parts(sample: object) -> tuple[float, np.ndarray]:
    if isinstance(sample, GyroSample):
        return _timestamp_seconds(sample.timestamp), _vector3(sample.angular_velocity, "angular_velocity")
    values = tuple(sample)  # type: ignore[arg-type]
    if len(values) != 2:
        raise ValueError("sample must be (timestamp, angular_velocity)")
    return _timestamp_seconds(values[0]), _vector3(values[1], "angular_velocity")


def _normalise_samples(samples: Iterable[object]) -> list[GyroSample]:
    parsed = [GyroSample(timestamp, vector) for timestamp, vector in (_sample_parts(s) for s in samples)]
    parsed.sort(key=lambda sample: sample.timestamp)
    unique: list[GyroSample] = []
    for sample in parsed:
        if unique and sample.timestamp == unique[-1].timestamp:
            unique[-1] = sample
        else:
            unique.append(sample)
    return unique


def stationary_bias(
    samples: Iterable[object],
    duration: float = 3.0,
    variation_threshold: float = 0.05,
    maximum_rate: float = 0.25,
) -> tuple[np.ndarray | None, str]:
    """Estimate a zero-rate bias, rejecting a moving calibration window.

    Returns ``(bias, "calibrated")`` for a stationary window and
    ``(None, reason)`` otherwise.  Rates are in rad/s and timestamps are in
    seconds.  The standard deviation threshold catches movement; the rate
    threshold also rejects a steady but rotating calibration window.
    """

    if duration <= 0.0 or variation_threshold < 0.0 or maximum_rate < 0.0:
        raise ValueError("calibration thresholds must be non-negative")
    normalised = _normalise_samples(samples)
    if len(normalised) < 2:
        return None, "insufficient_samples"
    elapsed = normalised[-1].timestamp - normalised[0].timestamp
    if elapsed < duration:
        return None, "insufficient_duration"
    values = np.asarray([sample.angular_velocity for sample in normalised], dtype=float)
    bias = values.mean(axis=0)
    variation = values.std(axis=0)
    if not np.all(np.isfinite(bias)) or not np.all(np.isfinite(variation)):
        return None, "invalid_samples"
    if float(np.max(variation)) > variation_threshold or float(np.linalg.norm(bias)) > maximum_rate:
        return None, "motion"
    return bias, "calibrated"


def _interpolate(samples: Sequence[GyroSample], timestamp: float) -> np.ndarray | None:
    times = np.asarray([sample.timestamp for sample in samples], dtype=float)
    if timestamp < times[0] or timestamp > times[-1]:
        return None
    index = int(np.searchsorted(times, timestamp, side="left"))
    if index == 0:
        return samples[0].angular_velocity.copy()
    if index == len(samples):
        return samples[-1].angular_velocity.copy()
    if times[index] == timestamp:
        return samples[index].angular_velocity.copy()
    before = samples[index - 1]
    after = samples[index]
    fraction = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
    return before.angular_velocity + fraction * (after.angular_velocity - before.angular_velocity)


def integrate_angular_velocity(
    samples: Iterable[object],
    t0: object,
    t1: object,
    *,
    bias: Sequence[float] | np.ndarray = np.zeros(3),
    rotation_camera_from_gyro: Sequence[Sequence[float]] | np.ndarray = np.eye(3),
    scale: float = 1.0,
    max_gap: float = 0.25,
) -> np.ndarray | None:
    """Integrate a timestamped gyro interval into a new-camera-to-old matrix.

    Sensor samples are angular rates in rad/s.  The returned matrix follows
    the camera-to-world increment convention: for a forward interval it is
    the product of ``exp([omega_camera * dt]x)`` terms and maps coordinates in
    the newer camera frame into the older camera frame.  Intervals outside
    the sample bounds, or containing a gap larger than ``max_gap``, return
    ``None``.
    """

    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError("scale must be a positive finite number")
    if not np.isfinite(max_gap) or max_gap <= 0.0:
        raise ValueError("max_gap must be a positive finite number")
    start = _timestamp_seconds(t0)
    end = _timestamp_seconds(t1)
    sensor_bias = _vector3(bias, "bias")
    mount = _rotation_matrix(rotation_camera_from_gyro)
    normalised = _normalise_samples(samples)
    if not normalised:
        return None
    if start == end:
        return np.eye(3) if normalised[0].timestamp <= start <= normalised[-1].timestamp else None
    reverse = end < start
    if reverse:
        start, end = end, start
    if start < normalised[0].timestamp or end > normalised[-1].timestamp:
        return None
    if any(
        right.timestamp - left.timestamp > max_gap
        and left.timestamp < end
        and right.timestamp > start
        for left, right in zip(normalised, normalised[1:])
    ):
        return None

    start_rate = _interpolate(normalised, start)
    end_rate = _interpolate(normalised, end)
    if start_rate is None or end_rate is None:
        return None
    points: list[tuple[float, np.ndarray]] = [(start, start_rate)]
    points.extend(
        (sample.timestamp, sample.angular_velocity.copy())
        for sample in normalised
        if start < sample.timestamp < end
    )
    points.append((end, end_rate))

    result = np.eye(3)
    for (left_time, left_rate), (right_time, right_rate) in zip(points, points[1:]):
        dt = right_time - left_time
        if dt <= 0.0:
            continue
        if dt > max_gap:
            return None
        midpoint_rate = 0.5 * (left_rate + right_rate)
        camera_rate = mount @ ((midpoint_rate - sensor_bias) * scale)
        result = result @ so3_exp(camera_rate * dt)
    if reverse:
        result = result.T
    return result


class Gyro:
    """Threaded MPU6050 gyro reader with optional camera rotation assistance."""

    def __init__(
        self,
        device: str = "/dev/i2c-1",
        address: int = 0x68,
        range_dps: int = 500,
        rotation_camera_from_gyro: Sequence[Sequence[float]] | np.ndarray = np.eye(3),
        enabled: bool = False,
        *,
        mounting_validated: bool | None = None,
        scale: float = 1.0,
        time_offset: float = 0.0,
        sample_hz: float = 100.0,
        dlpf_config: int = 3,
        calibration_duration: float = 3.0,
        calibration_variation_threshold: float = 0.05,
        calibration_max_rate: float = 0.25,
        max_gap: float = 0.25,
        history_seconds: float = 30.0,
        retry_interval: float = 1.0,
    ) -> None:
        if not isinstance(device, str) or not device:
            raise ValueError("device must be a non-empty path")
        if not isinstance(address, int) or not 0 <= address <= 0x7F:
            raise ValueError("address must be a 7-bit I2C address")
        if range_dps not in GYRO_RANGES_DPS:
            raise ValueError(f"range_dps must be one of {GYRO_RANGES_DPS}")
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("scale must be a positive finite number")
        if not np.isfinite(time_offset):
            raise ValueError("time_offset must be finite")
        if not np.isfinite(sample_hz) or sample_hz <= 0.0:
            raise ValueError("sample_hz must be positive")
        if not 0 <= dlpf_config <= 7:
            raise ValueError("dlpf_config must fit the MPU6050 CONFIG register")
        if calibration_duration <= 0.0 or calibration_variation_threshold < 0.0 or calibration_max_rate < 0.0:
            raise ValueError("calibration settings are invalid")
        if max_gap <= 0.0 or history_seconds <= max_gap or retry_interval <= 0.0:
            raise ValueError("history_seconds, max_gap, and retry_interval are invalid")

        self.device = device
        self.address = address
        self._requested_range_dps = range_dps
        self._range_dps = range_dps
        self._rotation = _rotation_matrix(rotation_camera_from_gyro)
        self._enabled = bool(enabled)
        self._mounting_validated = bool(mounting_validated) if mounting_validated is not None else False
        self._scale = float(scale)
        self._time_offset = float(time_offset)
        self._sample_hz = float(sample_hz)
        self._dlpf_config = int(dlpf_config)
        self._calibration_duration = float(calibration_duration)
        self._calibration_variation_threshold = float(calibration_variation_threshold)
        self._calibration_max_rate = float(calibration_max_rate)
        self._max_gap = float(max_gap)
        self._history_seconds = float(history_seconds)
        self._retry_interval = float(retry_interval)

        self._fd = -1
        self._who_am_i: int | None = None
        self._configured = False
        self._state = "not_started"
        self._last_error: str | None = None
        self._sample_count = 0
        self._error_count = 0
        self._samples: deque[GyroSample] = deque()
        self._calibration_samples: deque[GyroSample] = deque()
        self._bias: np.ndarray | None = None
        self._calibration_variation = np.zeros(3, dtype=float)
        self._motion_detected = False

        self._lock = threading.RLock()
        self._io_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def enabled(self) -> bool:
        with self._lock:
            return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        with self._lock:
            self._enabled = bool(value)

    @property
    def scale(self) -> float:
        with self._lock:
            return self._scale

    @scale.setter
    def scale(self, value: float) -> None:
        self.set_scale(value)

    @property
    def time_offset(self) -> float:
        with self._lock:
            return self._time_offset

    @time_offset.setter
    def time_offset(self, value: float) -> None:
        self.set_time_offset(value)

    def set_mounting_validated(self, validated: bool) -> None:
        """Allow fusion only after an operator validates the mount rotation."""

        with self._lock:
            self._mounting_validated = bool(validated)

    def set_time_offset(self, offset: float) -> None:
        if not np.isfinite(offset):
            raise ValueError("time offset must be finite")
        with self._lock:
            if offset == self._time_offset:
                return
            self._time_offset = float(offset)
            self._samples.clear()
            self._calibration_samples.clear()
            self._bias = None
            self._calibration_variation = np.zeros(3, dtype=float)
            self._state = "calibrating"

    def set_scale(self, scale: float) -> None:
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("scale must be a positive finite number")
        with self._lock:
            if scale == self._scale:
                return
            self._scale = float(scale)
            # Samples and bias are already scale-corrected, so a live tuning
            # change starts a fresh calibration window instead of mixing units.
            self._samples.clear()
            self._calibration_samples.clear()
            self._bias = None
            self._calibration_variation = np.zeros(3, dtype=float)
            self._state = "calibrating"

    def _write_fd(self, fd: int, register: int, value: int) -> None:
        payload = bytes((register, value))
        written = os.write(fd, payload)
        if written != len(payload):
            raise OSError(f"short I2C write to 0x{register:02x}")

    def _read_fd(self, fd: int, register: int, length: int) -> bytes:
        selected = bytes((register,))
        if os.write(fd, selected) != len(selected):
            raise OSError(f"failed to select MPU6050 register 0x{register:02x}")
        payload = os.read(fd, length)
        if len(payload) != length:
            raise OSError(f"short I2C read at 0x{register:02x}")
        return payload

    def _configure_fd(self, fd: int) -> tuple[int, int]:
        expected_fs_bits = GYRO_RANGES_DPS.index(self._requested_range_dps) << 3
        divider = max(0, min(255, int(round(1000.0 / self._sample_hz)) - 1))
        self._write_fd(fd, REG_PWR_MGMT_1, 0x01)  # PLL with X gyro reference.
        self._write_fd(fd, REG_SMPLRT_DIV, divider)
        self._write_fd(fd, REG_CONFIG, self._dlpf_config)
        self._write_fd(fd, REG_GYRO_CONFIG, expected_fs_bits)

        who_am_i = self._read_fd(fd, REG_WHO_AM_I, 1)[0]
        if who_am_i not in VALID_WHO_AM_I:
            raise OSError(f"unexpected MPU6050 WHO_AM_I 0x{who_am_i:02x}")
        dlpf_readback = self._read_fd(fd, REG_CONFIG, 1)[0] & 0x07
        gyro_readback = self._read_fd(fd, REG_GYRO_CONFIG, 1)[0] & 0x18
        if dlpf_readback != self._dlpf_config or gyro_readback != expected_fs_bits:
            self._write_fd(fd, REG_CONFIG, self._dlpf_config)
            self._write_fd(fd, REG_GYRO_CONFIG, expected_fs_bits)
            dlpf_readback = self._read_fd(fd, REG_CONFIG, 1)[0] & 0x07
            gyro_readback = self._read_fd(fd, REG_GYRO_CONFIG, 1)[0] & 0x18
            if dlpf_readback != self._dlpf_config or gyro_readback != expected_fs_bits:
                raise OSError("MPU6050 configuration readback mismatch")
        actual_range = GYRO_RANGES_DPS[(gyro_readback >> 3) & 0x03]
        return who_am_i, actual_range

    def _connect_once(self) -> bool:
        with self._io_lock:
            with self._lock:
                if self._fd >= 0:
                    return True
            fd = -1
            try:
                fd = os.open(self.device, os.O_RDWR | getattr(os, "O_CLOEXEC", 0))
                fcntl.ioctl(fd, I2C_SLAVE, self.address)
                who_am_i, actual_range = self._configure_fd(fd)
            except (OSError, ValueError) as error:
                if fd >= 0:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                with self._lock:
                    self._state = "unavailable"
                    self._configured = False
                    self._last_error = str(error)
                    self._error_count += 1
                return False
            with self._lock:
                self._fd = fd
                self._who_am_i = who_am_i
                self._range_dps = actual_range
                self._configured = True
                self._last_error = None
                if self._bias is None:
                    self._state = "calibrating"
            return True

    def _drop_connection(self, error: Exception) -> None:
        with self._io_lock:
            with self._lock:
                fd, self._fd = self._fd, -1
                self._configured = False
                self._samples.clear()
                self._calibration_samples.clear()
                self._bias = None
                self._calibration_variation = np.zeros(3, dtype=float)
                self._state = "unavailable"
                self._last_error = str(error)
                self._error_count += 1
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass

    def _read_sample(self) -> GyroSample:
        with self._io_lock:
            with self._lock:
                fd = self._fd
                lsb_per_dps = GYRO_LSB_PER_DPS[self._range_dps]
                scale = self._scale
                offset = self._time_offset
            if fd < 0:
                raise OSError("MPU6050 is not connected")
            # Deliberately read only 0x43..0x48: no accelerometer registers.
            raw = struct.unpack(">hhh", self._read_fd(fd, REG_GYRO_OUT, 6))
        angular_velocity = np.asarray(raw, dtype=float) / lsb_per_dps
        angular_velocity *= np.pi / 180.0 * scale
        return GyroSample(time.time() + offset, angular_velocity)

    def _record_sample(self, sample: GyroSample) -> None:
        with self._lock:
            self._samples.append(sample)
            self._sample_count += 1
            newest = sample.timestamp
            while self._samples and newest - self._samples[0].timestamp > self._history_seconds:
                self._samples.popleft()

            if self._bias is not None:
                return
            self._calibration_samples.append(sample)
            while (
                len(self._calibration_samples) >= 2
                and newest - self._calibration_samples[1].timestamp > self._calibration_duration
            ):
                self._calibration_samples.popleft()
            bias, reason = stationary_bias(
                self._calibration_samples,
                duration=self._calibration_duration,
                variation_threshold=self._calibration_variation_threshold,
                maximum_rate=self._calibration_max_rate,
            )
            if reason == "calibrated" and bias is not None:
                values = np.asarray([item.angular_velocity for item in self._calibration_samples])
                self._bias = bias
                self._calibration_variation = values.std(axis=0)
                self._state = "calibrated"
                self._motion_detected = False
            elif reason == "motion":
                self._calibration_samples.clear()
                self._state = "calibration_motion"
                self._motion_detected = True

    def inject_sample(self, timestamp: float, angular_velocity: Sequence[float] | np.ndarray) -> None:
        """Inject a rad/s sample for deterministic tests or replay."""

        self._record_sample(GyroSample(_timestamp_seconds(timestamp), _vector3(angular_velocity, "angular_velocity")))

    def recording_snapshot(self) -> dict[str, object]:
        """Return the bounded capture in a form suitable for ``numpy.savez``.

        ``samples`` has columns ``timestamp_s, gx_rad_s, gy_rad_s, gz_rad_s``.
        Hardware timestamps already include ``time_offset_s`` and hardware
        rates already include the configured ``scale``; the raw rate samples
        still include the bias, which is returned separately in the same
        corrected rad/s units.  Offline consumers must not apply either
        correction again.  An unavailable or uncalibrated reader returns an
        empty ``(0, 4)`` sample array and a three-element NaN bias.
        """

        with self._lock:
            samples = np.empty((len(self._samples), 4), dtype=np.float64)
            for index, sample in enumerate(self._samples):
                samples[index, 0] = sample.timestamp
                samples[index, 1:] = sample.angular_velocity
            bias = (
                self._bias.copy()
                if self._bias is not None
                else np.full(3, np.nan, dtype=np.float64)
            )
            return {
                "samples": samples,
                "sample_columns": np.asarray(
                    ["timestamp_s", "gx_rad_s", "gy_rad_s", "gz_rad_s"]
                ),
                "bias_rad_s": bias,
                "scale": float(self._scale),
                "time_offset_s": float(self._time_offset),
                "range_dps": int(self._range_dps),
                "calibrated": bool(self._bias is not None),
                "scale_applied": True,
                "time_offset_applied": True,
                "bias_subtracted": False,
                "angular_velocity_units": "rad/s",
                "timestamp_units": "Unix seconds",
            }

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            self._state = "starting"
            self._thread = threading.Thread(target=self._run, name="mpu6050-gyro", daemon=True)
            thread = self._thread
        self._connect_once()
        thread.start()

    def _run(self) -> None:
        period = 1.0 / self._sample_hz
        next_read = time.monotonic()
        next_retry = next_read
        while not self._stop.is_set():
            with self._lock:
                connected = self._fd >= 0
            now = time.monotonic()
            if not connected:
                if now < next_retry:
                    self._stop.wait(next_retry - now)
                    continue
                if not self._connect_once():
                    next_retry = time.monotonic() + self._retry_interval
                    continue
                next_read = time.monotonic()
            wait = next_read - time.monotonic()
            if wait > 0.0:
                if self._stop.wait(wait):
                    break
            try:
                self._record_sample(self._read_sample())
            except (OSError, ValueError, struct.error) as error:
                self._drop_connection(error)
                next_retry = time.monotonic() + self._retry_interval
                continue
            next_read += period
            if next_read < time.monotonic() - period:
                next_read = time.monotonic()

    def close(self) -> None:
        self._stop.set()
        with self._lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, 2.0 / self._sample_hz))
        with self._io_lock:
            with self._lock:
                fd, self._fd = self._fd, -1
                self._configured = False
                self._samples.clear()
                self._calibration_samples.clear()
                self._bias = None
                self._calibration_variation = np.zeros(3, dtype=float)
                self._state = "closed"
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass

    def relative_rotation(self, t0: object, t1: object) -> np.ndarray | None:
        """Return new-camera-to-old rotation for a camera timestamp interval."""

        with self._lock:
            latest = self._samples[-1].timestamp if self._samples else None
            if not (
                self._enabled
                and self._mounting_validated
                and self._configured
                and self._bias is not None
                and latest is not None
                and abs(time.time() + self._time_offset - latest) <= self._max_gap
            ):
                return None
            samples = tuple(self._samples)
            bias = self._bias.copy()
            rotation = self._rotation.copy()
            scale = 1.0  # samples are already converted and tuned in _read_sample.
            max_gap = self._max_gap
        # Retain interpolation neighbours without re-validating 30 s of history
        # for every short frame increment on the Pi.
        start, end = sorted((_timestamp_seconds(t0), _timestamp_seconds(t1)))
        times = [sample.timestamp for sample in samples]
        samples = samples[max(0, bisect_left(times, start)-1):bisect_right(times, end)+1]
        return integrate_angular_velocity(
            samples,
            t0,
            t1,
            bias=bias,
            rotation_camera_from_gyro=rotation,
            scale=scale,
            max_gap=max_gap,
        )

    def status(self) -> dict[str, object]:
        """Return a JSON-serializable snapshot of connection and calibration state."""

        with self._lock:
            calibration_elapsed = 0.0
            if len(self._calibration_samples) >= 2:
                calibration_elapsed = (
                    self._calibration_samples[-1].timestamp
                    - self._calibration_samples[0].timestamp
                )
            latest = self._samples[-1].timestamp if self._samples else None
            oldest = self._samples[0].timestamp if self._samples else None
            bias = self._bias.tolist() if self._bias is not None else None
            fresh = latest is not None and abs(time.time() + self._time_offset - latest) <= self._max_gap
            return {
                "device": self.device,
                "address": self.address,
                "connected": self._fd >= 0,
                "configured": self._configured,
                "device_id": self._who_am_i,
                "device_id_valid": self._who_am_i in VALID_WHO_AM_I,
                "requested_range_dps": self._requested_range_dps,
                "range_dps": self._range_dps,
                "gyro_scale_lsb_per_dps": GYRO_LSB_PER_DPS[self._range_dps],
                "scale": self._scale,
                "sample_hz": self._sample_hz,
                "dlpf_config": self._dlpf_config,
                "time_offset_s": self._time_offset,
                "rotation_camera_from_gyro": self._rotation.tolist(),
                "enabled": self._enabled,
                "mounting_validated": self._mounting_validated,
                "fusion_ready": bool(
                    self._enabled
                    and self._mounting_validated
                    and self._configured
                    and self._bias is not None
                    and fresh
                ),
                "state": self._state,
                "status": self._state,
                "calibrated": self._bias is not None,
                "bias_rad_s": bias,
                "calibration_variation_rad_s": self._calibration_variation.tolist(),
                "calibration_samples": len(self._calibration_samples),
                "calibration_elapsed_s": calibration_elapsed,
                "motion_detected": self._motion_detected,
                "sample_count": self._sample_count,
                "oldest_sample_time": oldest,
                "latest_sample_time": latest,
                "error_count": self._error_count,
                "last_error": self._last_error,
            }

__all__ = [
    "Gyro",
    "GyroSample",
    "integrate_angular_velocity",
    "so3_exp",
    "stationary_bias",
]
