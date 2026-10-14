import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from duojin02_teleop.keyboard_teleop_core import KeyboardTeleopCore


def create_core() -> KeyboardTeleopCore:
    return KeyboardTeleopCore(
        linear_speed=1.0,
        angular_speed=1.0,
        speed_step=0.1,
        turn_step=0.1,
        accel_limit_linear=100.0,
        decel_limit_linear=100.0,
        accel_limit_angular=100.0,
        decel_limit_angular=100.0,
    )


def test_hold_persists_until_repeat_timeout():
    core = create_core()
    start_time = 100.0

    assert core.handle_key_press('w', start_time) is True

    command = core.snapshot(start_time + 0.5)
    assert command.linear_x == 1.0
    assert command.active_keys == ['w']


def test_repeat_expiry_causes_smooth_deceleration():
    core = KeyboardTeleopCore(
        linear_speed=1.0,
        angular_speed=1.0,
        speed_step=0.1,
        turn_step=0.1,
        accel_limit_linear=2.0,
        decel_limit_linear=3.0,
        accel_limit_angular=6.0,
        decel_limit_angular=8.0,
    )
    start_time = 100.0

    assert core.handle_key_press('w', start_time) is True
    active_command = core.snapshot(start_time)
    assert active_command.linear_x == 1.0

    assert core.snapshot(start_time + 0.5).linear_x == 1.0
    decelerating_command = core.snapshot(start_time + 0.7)
    assert math.isclose(decelerating_command.linear_x, 0.4, rel_tol=0.0, abs_tol=1e-9)
    assert decelerating_command.active_keys == []

    stopped_command = core.snapshot(start_time + 1.0)
    assert stopped_command.linear_x == 0.0


def test_emergency_stop_zeros_motion_immediately():
    core = create_core()
    start_time = 100.0

    assert core.handle_key_press('w', start_time) is True
    command = core.snapshot(start_time + 0.1)
    assert command.linear_x == 1.0

    assert core.handle_key_press('x', start_time + 0.2) is True
    stopped_command = core.snapshot(start_time + 0.2)
    assert stopped_command.linear_x == 0.0
    assert stopped_command.angular_z == 0.0
    assert stopped_command.active_keys == []


def test_speed_adjustment_stays_non_negative():
    core = create_core()
    start_time = 100.0

    assert core.handle_key_press('k', start_time) is True
    assert core.handle_key_press('k', start_time + 0.1) is True
    assert core.handle_key_press('k', start_time + 0.2) is True
    assert core.handle_key_press('k', start_time + 0.3) is True
    assert core.handle_key_press('k', start_time + 0.4) is True
    assert core.handle_key_press('k', start_time + 0.5) is True
    assert core.handle_key_press('k', start_time + 0.6) is True
    assert core.handle_key_press('k', start_time + 0.7) is True
    assert core.handle_key_press('k', start_time + 0.8) is True
    assert core.handle_key_press('k', start_time + 0.9) is True
    assert core.handle_key_press('k', start_time + 1.0) is True

    assert core.handle_key_press('l', start_time + 1.1) is True
    assert core.handle_key_press('l', start_time + 1.2) is True
    assert core.handle_key_press('l', start_time + 1.3) is True
    assert core.handle_key_press('l', start_time + 1.4) is True
    assert core.handle_key_press('l', start_time + 1.5) is True
    assert core.handle_key_press('l', start_time + 1.6) is True
    assert core.handle_key_press('l', start_time + 1.7) is True
    assert core.handle_key_press('l', start_time + 1.8) is True
    assert core.handle_key_press('l', start_time + 1.9) is True
    assert core.handle_key_press('l', start_time + 2.0) is True
    assert core.handle_key_press('l', start_time + 2.1) is True

    status = core.status()
    assert status.linear_speed == 0.0
    assert status.angular_speed == 0.0

    assert core.handle_key_press('i', start_time + 2.2) is True
    assert core.handle_key_press('o', start_time + 2.3) is True

    status = core.status()
    assert status.linear_speed == 0.1
    assert status.angular_speed == 0.1


def test_speed_adjustment_respects_configured_maximums():
    core = KeyboardTeleopCore(
        linear_speed=0.8,
        angular_speed=0.7,
        speed_step=0.1,
        turn_step=0.2,
        max_linear_speed=0.6,
        max_angular_speed=0.6,
        accel_limit_linear=100.0,
        decel_limit_linear=100.0,
        accel_limit_angular=100.0,
        decel_limit_angular=100.0,
    )
    start_time = 100.0

    status = core.status()
    assert status.linear_speed == 0.6
    assert status.angular_speed == 0.6

    assert core.handle_key_press('i', start_time) is True
    assert core.handle_key_press('o', start_time + 0.1) is True

    status = core.status()
    assert status.linear_speed == 0.6
    assert status.angular_speed == 0.6


def test_slew_rate_limits_acceleration_between_snapshots():
    core = KeyboardTeleopCore(
        linear_speed=1.0,
        angular_speed=1.0,
        speed_step=0.1,
        turn_step=0.1,
        accel_limit_linear=2.0,
        decel_limit_linear=3.0,
        accel_limit_angular=6.0,
        decel_limit_angular=8.0,
    )
    start_time = 100.0

    assert core.handle_key_press('w', start_time) is True

    first_command = core.snapshot(start_time)
    assert first_command.linear_x == 1.0

    assert core.snapshot(start_time + 0.5).linear_x == 1.0
    decelerating_command = core.snapshot(start_time + 0.52)
    assert math.isclose(decelerating_command.linear_x, 0.94, rel_tol=0.0, abs_tol=1e-9)

    assert core.handle_key_press('w', start_time + 0.53) is True
    accelerating_command = core.snapshot(start_time + 0.54)
    assert math.isclose(accelerating_command.linear_x, 0.98, rel_tol=0.0, abs_tol=1e-9)


def test_combined_keys_produce_combined_motion():
    core = create_core()
    start_time = 100.0

    assert core.handle_key_press('w', start_time) is True
    assert core.handle_key_press('a', start_time + 0.01) is True

    command = core.snapshot(start_time + 0.1)
    assert command.linear_x == 1.0
    assert command.linear_y == 1.0
    assert command.active_keys == ['a', 'w']


def test_stale_key_timeout_expires_pressed_keys():
    core = create_core()
    start_time = 100.0

    assert core.handle_key_press('w', start_time) is True

    active_command = core.snapshot(start_time + 0.1, stale_key_timeout_sec=0.5)
    assert active_command.linear_x == 1.0
    assert active_command.active_keys == ['w']

    expired_command = core.snapshot(start_time + 0.7, stale_key_timeout_sec=0.5)
    assert expired_command.linear_x == 0.0
    assert expired_command.active_keys == []


def test_keys_expire_independently_when_other_movement_is_repeated():
    core = create_core()
    core.handle_key_press('w', 100.0)
    core.handle_key_press('a', 100.0)
    core.snapshot(100.1)
    core.handle_key_press('a', 100.4)

    command = core.snapshot(100.6)
    assert command.linear_x == 0.0
    assert command.linear_y == 1.0
    assert command.active_keys == ['a']


def test_speed_keys_do_not_refresh_movement():
    core = create_core()
    core.handle_key_press('w', 100.0)
    core.snapshot(100.1)
    core.handle_key_press('i', 100.4)

    command = core.snapshot(100.6)
    assert command.linear_x == 0.0
    assert command.active_keys == []
    assert command.linear_speed == 1.1


def test_stop_request_is_delivered_once_even_when_keyboard_is_idle():
    core = create_core()
    assert core.handle_key_press('x', 100.0)
    command = core.snapshot(100.1)
    assert command.stop_requested
    assert command.linear_x == command.linear_y == command.angular_z == 0.0
    assert not core.snapshot(100.2).stop_requested


def test_opposing_held_keys_preserve_manual_ownership_at_zero_speed():
    core = create_core()
    core.handle_key_press('w', 100.0)
    core.handle_key_press('s', 100.0)
    command = core.snapshot(100.1)
    assert command.linear_x == 0.0
    assert command.active_keys == ['s', 'w']


@pytest.mark.parametrize('timeout', [0.0, -0.5, math.nan, math.inf, -math.inf])
def test_snapshot_rejects_invalid_expiry_instead_of_republishing_cached_motion(timeout):
    core = create_core()
    core.handle_key_press('w', 100.0)
    assert core.snapshot(100.1).linear_x == 1.0
    with pytest.raises(ValueError, match='stale_key_timeout_sec'):
        core.snapshot(100.2, stale_key_timeout_sec=timeout)
    assert core.snapshot(100.7).linear_x == 0.0
