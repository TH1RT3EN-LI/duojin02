import math

DEFAULT_CMD_VEL_TOPIC = "/cmd_vel_keyboard"

DEFAULT_LINEAR_SPEED = 0.3
DEFAULT_ANGULAR_SPEED = 0.3
DEFAULT_MAX_LINEAR_SPEED = 0.6
DEFAULT_MAX_ANGULAR_SPEED = 0.6
DEFAULT_LINEAR_SPEED_STEP = 0.05
DEFAULT_ANGULAR_SPEED_STEP = 0.1

ACCEL_LIMIT_LINEAR = 2.0
DECEL_LIMIT_LINEAR = 3.0
ACCEL_LIMIT_ANGULAR = 6.0
DECEL_LIMIT_ANGULAR = 8.0

DEFAULT_PUBLISH_RATE = 120.0
DEFAULT_REPEAT_TIMEOUT_SEC = 0.50
DEFAULT_TTY_DEVICE_PATH = ""


KEYBOARD_TELEOP_PARAMETER_DEFAULTS = {
    "cmd_vel_topic": DEFAULT_CMD_VEL_TOPIC,
    "speed": DEFAULT_LINEAR_SPEED,
    "turn": DEFAULT_ANGULAR_SPEED,
    "max_speed": DEFAULT_MAX_LINEAR_SPEED,
    "max_turn": DEFAULT_MAX_ANGULAR_SPEED,
    "speed_step": DEFAULT_LINEAR_SPEED_STEP,
    "turn_step": DEFAULT_ANGULAR_SPEED_STEP,
    "publish_rate": DEFAULT_PUBLISH_RATE,
    "repeat_timeout_sec": DEFAULT_REPEAT_TIMEOUT_SEC,
    "tty_device_path": DEFAULT_TTY_DEVICE_PATH,
}


def keyboard_teleop_parameter_defaults() -> dict[str, float | str]:
    return dict(KEYBOARD_TELEOP_PARAMETER_DEFAULTS)


def validate_keyboard_parameters(parameters: dict[str, float | str]) -> None:
    for name in ("repeat_timeout_sec", "publish_rate"):
        value = float(parameters[name])
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and greater than zero")
    for name in ("speed", "turn", "max_speed", "max_turn", "speed_step", "turn_step"):
        value = float(parameters[name])
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and nonnegative")
