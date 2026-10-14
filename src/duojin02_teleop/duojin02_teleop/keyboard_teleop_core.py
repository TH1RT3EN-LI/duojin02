import math
import threading
import time
from dataclasses import dataclass, field
from duojin02_teleop.keyboard_teleop_defaults import (
    ACCEL_LIMIT_ANGULAR,
    ACCEL_LIMIT_LINEAR,
    DECEL_LIMIT_ANGULAR,
    DECEL_LIMIT_LINEAR,
    DEFAULT_IDLE_TIMEOUT_SEC,
)


MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0, 0.0),
    'd': (0.0, -1.0, 0.0, 0.0),
    'q': (0.0, 0.0, 0.0, 1.0),
    'e': (0.0, 0.0, 0.0, -1.0),
    'x': (0.0, 0.0, 0.0, 0.0),
}

SPEED_BINDINGS = {
    'i': (1.0, 0.0),
    'k': (-1.0, 0.0),
    'o': (0.0, 1.0),
    'l': (0.0, -1.0),
}


@dataclass(frozen=True)
class TeleopCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_z: float = 0.0
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    active_keys: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class KeyboardTeleopStatus:
    linear_speed: float = 0.0
    angular_speed: float = 0.0
    active_keys: list[str] = field(default_factory=list)


class KeyboardTeleopCore:
    def __init__(
        self,
        linear_speed: float,
        angular_speed: float,
        speed_step: float,
        turn_step: float,
        max_linear_speed: float | None = None,
        max_angular_speed: float | None = None,
        accel_limit_linear: float = ACCEL_LIMIT_LINEAR,
        decel_limit_linear: float = DECEL_LIMIT_LINEAR,
        accel_limit_angular: float = ACCEL_LIMIT_ANGULAR,
        decel_limit_angular: float = DECEL_LIMIT_ANGULAR,
        idle_timeout_sec: float = DEFAULT_IDLE_TIMEOUT_SEC,
    ) -> None:
        self._lock = threading.Lock()
        self._pressed_move_keys: set[str] = set()
        self._move_key_last_seen: dict[str, float] = {}

        self._max_linear_speed = self._coerce_max_speed(max_linear_speed)
        self._max_angular_speed = self._coerce_max_speed(max_angular_speed)
        self._linear_speed = self._clamp_speed(linear_speed, self._max_linear_speed)
        self._angular_speed = self._clamp_speed(angular_speed, self._max_angular_speed)
        self._speed_step = max(0.0, float(speed_step))
        self._turn_step = max(0.0, float(turn_step))

        self._accel_limit_linear = max(1e-6, float(accel_limit_linear))
        self._decel_limit_linear = max(1e-6, float(decel_limit_linear))
        self._accel_limit_angular = max(1e-6, float(accel_limit_angular))
        self._decel_limit_angular = max(1e-6, float(decel_limit_angular))
        self._idle_timeout_sec = max(0.0, float(idle_timeout_sec))

        self._current_linear_x = 0.0
        self._current_linear_y = 0.0
        self._current_linear_z = 0.0
        self._current_angular_z = 0.0

        self._last_input_time = time.monotonic()
        self._last_update_time: float | None = None

    def handle_key_press(self, key: str, now: float | None = None) -> bool:
        normalized_key = self._normalize_key(key)
        if normalized_key is None:
            return False

        timestamp = self._coerce_now(now)
        with self._lock:
            self._last_input_time = timestamp

            if normalized_key == 'x':
                self._clear_move_keys_locked()
                self._zero_command_locked()
                self._last_update_time = timestamp
                return True

            if normalized_key in MOVE_BINDINGS:
                self._pressed_move_keys.add(normalized_key)
                self._move_key_last_seen[normalized_key] = timestamp
                return True

            if normalized_key in SPEED_BINDINGS:
                linear_step_direction, angular_step_direction = SPEED_BINDINGS[normalized_key]
                self._linear_speed = self._clamp_speed(
                    self._linear_speed + linear_step_direction * self._speed_step,
                    self._max_linear_speed,
                )
                self._angular_speed = self._clamp_speed(
                    self._angular_speed + angular_step_direction * self._turn_step,
                    self._max_angular_speed,
                )
                return True

        return False

    def handle_key_release(self, key: str, now: float | None = None) -> bool:
        normalized_key = self._normalize_key(key)
        if normalized_key is None or normalized_key not in MOVE_BINDINGS:
            return False

        timestamp = self._coerce_now(now)
        with self._lock:
            self._last_input_time = timestamp
            if normalized_key != 'x':
                self._pressed_move_keys.discard(normalized_key)
                self._move_key_last_seen.pop(normalized_key, None)
        return True

    def clear_move_keys(self, now: float | None = None) -> None:
        timestamp = self._coerce_now(now)
        with self._lock:
            self._clear_move_keys_locked()
            self._last_input_time = timestamp

    def emergency_stop(self, now: float | None = None) -> None:
        timestamp = self._coerce_now(now)
        with self._lock:
            self._clear_move_keys_locked()
            self._zero_command_locked()
            self._last_input_time = timestamp
            self._last_update_time = timestamp

    def expire_stale_move_keys(self, stale_after_sec: float, now: float | None = None) -> None:
        timeout_sec = max(0.0, float(stale_after_sec))
        timestamp = self._coerce_now(now)
        with self._lock:
            self._expire_stale_locked(timestamp, timeout_sec)

    def snapshot(
        self,
        now: float | None = None,
        stale_key_timeout_sec: float | None = None,
    ) -> TeleopCommand:
        timestamp = self._coerce_now(now)
        with self._lock:
            if stale_key_timeout_sec is not None:
                self._expire_stale_locked(timestamp, max(0.0, float(stale_key_timeout_sec)))

            idle_elapsed = timestamp - self._last_input_time
            if self._idle_timeout_sec > 0.0 and idle_elapsed > self._idle_timeout_sec:
                self._clear_move_keys_locked()

            target_linear_x, target_linear_y, target_linear_z, target_angular_z = self._target_vector_locked()
            target_linear_x *= self._linear_speed
            target_linear_y *= self._linear_speed
            target_linear_z *= self._linear_speed
            target_angular_z *= self._angular_speed

            if self._last_update_time is None:
                delta_time = 0.0
            else:
                delta_time = max(0.0, timestamp - self._last_update_time)

            self._current_linear_x = self._slew_axis(
                self._current_linear_x,
                target_linear_x,
                delta_time,
                self._accel_limit_linear,
                self._decel_limit_linear,
            )
            self._current_linear_y = self._slew_axis(
                self._current_linear_y,
                target_linear_y,
                delta_time,
                self._accel_limit_linear,
                self._decel_limit_linear,
            )
            self._current_linear_z = self._slew_axis(
                self._current_linear_z,
                target_linear_z,
                delta_time,
                self._accel_limit_linear,
                self._decel_limit_linear,
            )
            self._current_angular_z = self._slew_axis(
                self._current_angular_z,
                target_angular_z,
                delta_time,
                self._accel_limit_angular,
                self._decel_limit_angular,
            )
            self._last_update_time = timestamp

            return TeleopCommand(
                linear_x=self._current_linear_x,
                linear_y=self._current_linear_y,
                linear_z=self._current_linear_z,
                angular_z=self._current_angular_z,
                linear_speed=self._linear_speed,
                angular_speed=self._angular_speed,
                active_keys=self._sorted_keys(self._pressed_move_keys),
            )

    def status(self) -> KeyboardTeleopStatus:
        with self._lock:
            return KeyboardTeleopStatus(
                linear_speed=self._linear_speed,
                angular_speed=self._angular_speed,
                active_keys=self._sorted_keys(self._pressed_move_keys),
            )

    def zero_command(self) -> TeleopCommand:
        with self._lock:
            self._zero_command_locked()
            return TeleopCommand(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=0.0,
                linear_speed=self._linear_speed,
                angular_speed=self._angular_speed,
                active_keys=self._sorted_keys(self._pressed_move_keys),
            )

    @staticmethod
    def is_move_key(key: str) -> bool:
        normalized_key = KeyboardTeleopCore._normalize_key(key)
        return normalized_key in MOVE_BINDINGS if normalized_key is not None else False

    @staticmethod
    def is_speed_key(key: str) -> bool:
        normalized_key = KeyboardTeleopCore._normalize_key(key)
        return normalized_key in SPEED_BINDINGS if normalized_key is not None else False

    @staticmethod
    def _normalize_key(key: str) -> str | None:
        if not key:
            return None

        normalized_key = str(key).lower()
        if normalized_key in MOVE_BINDINGS or normalized_key in SPEED_BINDINGS:
            return normalized_key
        return None

    @staticmethod
    def _coerce_now(now: float | None) -> float:
        return time.monotonic() if now is None else float(now)

    @staticmethod
    def _coerce_max_speed(max_speed: float | None) -> float | None:
        if max_speed is None:
            return None
        return max(0.0, float(max_speed))

    @staticmethod
    def _clamp_speed(value: float, max_speed: float | None) -> float:
        clamped_value = max(0.0, float(value))
        if max_speed is None:
            return clamped_value
        return min(clamped_value, max_speed)

    @staticmethod
    def _slew_axis(
        current_value: float,
        target_value: float,
        delta_time: float,
        accel_limit: float,
        decel_limit: float,
    ) -> float:
        if delta_time <= 0.0:
            return target_value
        if math.fabs(current_value - target_value) <= 1e-9:
            return target_value

        applied_limit = accel_limit
        if current_value == 0.0:
            applied_limit = accel_limit if math.fabs(target_value) > 0.0 else decel_limit
        elif (current_value * target_value) < 0.0 or math.fabs(target_value) < math.fabs(current_value):
            applied_limit = decel_limit

        max_delta = applied_limit * delta_time
        delta_value = target_value - current_value
        if math.fabs(delta_value) <= max_delta:
            return target_value
        return current_value + math.copysign(max_delta, delta_value)

    def _clear_move_keys_locked(self) -> None:
        self._pressed_move_keys.clear()
        self._move_key_last_seen.clear()

    def _zero_command_locked(self) -> None:
        self._current_linear_x = 0.0
        self._current_linear_y = 0.0
        self._current_linear_z = 0.0
        self._current_angular_z = 0.0

    def _expire_stale_locked(self, now: float, timeout_sec: float) -> None:
        stale_keys = [
            key_name
            for key_name, seen_at in self._move_key_last_seen.items()
            if (now - seen_at) > timeout_sec
        ]
        for key_name in stale_keys:
            self._pressed_move_keys.discard(key_name)
            self._move_key_last_seen.pop(key_name, None)

    def _target_vector_locked(self) -> tuple[float, float, float, float]:
        target_linear_x = 0.0
        target_linear_y = 0.0
        target_linear_z = 0.0
        target_angular_z = 0.0

        for key_name in self._pressed_move_keys:
            move_linear_x, move_linear_y, move_linear_z, move_angular_z = MOVE_BINDINGS[key_name]
            target_linear_x += move_linear_x
            target_linear_y += move_linear_y
            target_linear_z += move_linear_z
            target_angular_z += move_angular_z

        return (
            max(-1.0, min(1.0, target_linear_x)),
            max(-1.0, min(1.0, target_linear_y)),
            max(-1.0, min(1.0, target_linear_z)),
            max(-1.0, min(1.0, target_angular_z)),
        )

    @staticmethod
    def _sorted_keys(keys: set[str]) -> list[str]:
        return sorted(keys)
