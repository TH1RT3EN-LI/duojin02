"""Exercise manual takeover through real DDS, TTY, Joy, mux, Guard and PTY serial.

Navigation is a controlled fresh Twist publisher. The sensors and controller
feedback are fixtures; this probe does not measure physical stopping distance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import tty

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
import rclpy
from sensor_msgs.msg import Joy
import yaml

from duojin02_bringup.safety_profile import mux_parameters, safety_defaults
from navigation_probe_fixture import ROOT, SensorFixture
from ros_isolation import configure_isolation


def run(args):
    domain = configure_isolation()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    fixture = SensorFixture()
    joy_pub = fixture.create_publisher(Joy, '/joy', 10)
    keyboard = []
    fixture.create_subscription(Twist, '/cmd_vel_keyboard',
        lambda message: keyboard.append((time.monotonic(), message.linear.x,
                                         message.linear.y, message.angular.z)), 10)
    processes, logs = [], []
    result = {'pass': False, 'checks': [], 'cleanup': [], 'metrics': {},
              'ros_domain_id': domain,
              'scope': 'Actual keyboard TTY node, teleop_twist_joy, DDS mux/Guard and PTY base; '
                       'controlled fresh navigation and sensor inputs, no physical robot'}
    mux_file = args.output_dir / 'mux.yaml'
    mux_file.write_text(yaml.safe_dump({'twist_mux': {'ros__parameters': mux_parameters(
        ROOT / 'src/duojin02_bringup/config/twist_mux.yaml')}}))
    result['command_timeout_s'] = safety_defaults()['command_timeout_s']
    result['binary_sha256'] = {
        name: hashlib.sha256(path.read_bytes()).hexdigest()
        for name, path in [('guard', args.guard_binary), ('driver', args.driver_binary)]}
    result['source_sha256'] = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in [
            ROOT / 'src/duojin02_bringup/config/safety.yaml',
            ROOT / 'src/duojin02_bringup/config/twist_mux.yaml',
            ROOT / 'src/duojin02_bringup/config/joy_teleop_normal.yaml',
            ROOT / 'src/duojin02_bringup/config/joy_teleop_slow.yaml',
            ROOT / 'src/duojin02_bringup/duojin02_bringup/safety_profile.py',
            ROOT / 'src/duojin02_teleop/duojin02_teleop/keyboard_teleop_node.py',
            ROOT / 'src/duojin02_teleop/duojin02_teleop/keyboard_teleop_core.py',
            ROOT / 'src/duojin02_teleop/duojin02_teleop/keyboard_teleop_defaults.py']}
    base_master, base_slave = os.openpty()
    tty.setraw(base_slave)
    os.set_blocking(base_master, False)
    fixture.master = base_master
    key_master, key_slave = os.openpty()
    safety = str(ROOT / 'src/duojin02_bringup/config/safety.yaml')

    def start(name, command, expected_returncode=0):
        log = (args.output_dir / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((name, process, expected_returncode))
        return process

    def check(name, condition):
        assert condition, {'case': name, 'serial': fixture.wire_packets[-8:],
                           'guard': fixture.guard_status[-1:], 'keyboard': keyboard[-8:],
                           'requested': fixture.requested[-8:]}
        result['checks'].append(name)

    def serial_equals(x):
        return bool(fixture.wire_packets) and fixture.wire_packets[-1][1] == (x, 0, 0)

    def pump(duration, *, nav=.24, joy_mode=None, keys=None):
        until = time.monotonic() + duration
        next_joy = next_key = 0.0
        while time.monotonic() < until:
            now = time.monotonic()
            if now >= next_joy:
                next_joy = now + .05
                message = Joy()
                message.header.stamp = fixture.get_clock().now().to_msg()
                message.axes = [0.0, .5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                message.buttons = [0] * 8
                if joy_mode in ('normal', 'both'):
                    message.buttons[5] = 1
                if joy_mode in ('slow', 'both'):
                    message.buttons[4] = 1
                joy_pub.publish(message)
            if keys is not None and now >= next_key:
                next_key = now + .1
                os.write(key_master, keys)
            fixture.pump(min(.01, max(0.0, until-time.monotonic())), nav=nav)

    def start_keyboard(name, expected_returncode=0):
        return start(name, [sys.executable, str(ROOT / 'src/duojin02_teleop/'
            'duojin02_teleop/keyboard_teleop_node.py'), '--ros-args',
            '-p', f'tty_device_path:={os.ttyname(key_slave)}'], expected_returncode)

    try:
        start('mux', ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args',
            '--params-file', str(mux_file), '-r', '/cmd_vel_out:=/cmd_vel_requested'])
        start('guard', [str(args.guard_binary), '--ros-args', '--params-file', safety,
            '-p', 'velocity_guard:use_sim_time:=true'])
        start('driver', [str(args.driver_binary), '--ros-args', '--params-file', safety,
            '-p', f'usart_port_name:={os.ttyname(base_slave)}', '-p', 'serial_baud_rate:=57600',
            '-p', 'use_sim_time:=true', '-r', '/cmd_vel:=/cmd_vel_base'])
        teleop_binary = Path(get_package_prefix('teleop_twist_joy')) / 'lib/teleop_twist_joy/teleop_node'
        for mode in ('normal', 'slow'):
            start('joy_' + mode, [str(teleop_binary), '--ros-args', '-r', f'__node:=teleop_{mode}',
                '--params-file', str(ROOT / f'src/duojin02_bringup/config/joy_teleop_{mode}.yaml'),
                '-r', f'/cmd_vel:=/cmd_vel_{mode}'])
        first_keyboard = start_keyboard('keyboard')
        pump(3.0)
        check('fresh_navigation_reaches_serial', serial_equals(240))
        check('idle_keyboard_publishes_nothing', keyboard == [])

        pump(.8, joy_mode='normal')
        check('joy_normal_deadman_overrides_active_navigation', serial_equals(200))
        check('idle_keyboard_does_not_starve_joy', keyboard == [])
        pump(.8, joy_mode='slow')
        check('joy_slow_deadman_overrides_active_navigation', serial_equals(75))
        pump(.8, joy_mode='both')
        check('normal_mode_wins_when_both_deadmen_held', serial_equals(200))
        pump(.8)
        check('released_joy_deadman_returns_to_fresh_navigation', serial_equals(240))

        pump(.8, joy_mode='normal', keys=b'w')
        check('held_keyboard_overrides_joy_and_navigation', serial_equals(300))
        count = len(keyboard)
        pump(.8, joy_mode='normal', keys=b'ws')
        check('held_opposing_keys_keep_manual_zero_ownership', serial_equals(0))
        check('held_zero_is_republished_while_operator_active',
              sum(value[1:] == (0.0, 0.0, 0.0) for value in keyboard[count:]) >= 20)
        pump(1.1)
        check('keyboard_expiry_releases_to_fresh_navigation', serial_equals(240))
        quiet_count = len(keyboard)
        pump(.7, joy_mode='normal')
        check('expired_keyboard_is_quiet_and_joy_can_take_over',
              len(keyboard) == quiet_count and serial_equals(200))
        pump(.7)
        check('manual_sources_release_to_navigation_again', serial_equals(240))

        count = len(keyboard)
        os.write(key_master, b'x')
        pump(.15)
        check('idle_x_sends_one_explicit_zero',
              len(keyboard) == count + 1 and keyboard[-1][1:] == (0.0, 0.0, 0.0))
        check('x_zero_reaches_serial', serial_equals(0))
        pump(.7)
        check('x_releases_after_lease_without_idle_heartbeat',
              len(keyboard) == count + 1 and serial_equals(240))

        pump(.7, keys=b'w')
        check('active_keyboard_before_orderly_exit', serial_equals(300))
        count = len(keyboard)
        os.write(key_master, b'\x03')
        pump(.2)
        check('active_keyboard_exit_publishes_final_zero',
              len(keyboard) > count and keyboard[-1][1:] == (0.0, 0.0, 0.0))
        check('active_exit_zero_reaches_serial', serial_equals(0))
        first_keyboard.wait(timeout=8)
        check('keyboard_orderly_exit_success', first_keyboard.returncode == 0)
        pump(.7)
        check('orderly_keyboard_exit_returns_to_fresh_navigation', serial_equals(240))

        idle_keyboard = start_keyboard('keyboard_idle_exit')
        pump(1.2, joy_mode='normal')
        check('idle_keyboard_again_does_not_interrupt_joy', serial_equals(200))
        count = len(keyboard)
        os.write(key_master, b'\x03')
        pump(.2, joy_mode='normal')
        idle_keyboard.wait(timeout=8)
        check('idle_keyboard_exit_does_not_publish_or_claim_control',
              len(keyboard) == count and serial_equals(200) and idle_keyboard.returncode == 0)
        pump(.7)
        check('idle_exit_does_not_prevent_navigation_after_deadman_release', serial_equals(240))

        pending_x_keyboard = start_keyboard('keyboard_pending_x_exit')
        pump(1.2, joy_mode='normal')
        count = len(keyboard)
        os.write(key_master, b'x\x03')
        pump(.15, joy_mode='normal')
        pending_x_keyboard.wait(timeout=8)
        check('x_immediately_followed_by_exit_still_sends_explicit_zero',
              len(keyboard) == count + 1 and keyboard[-1][1:] == (0.0, 0.0, 0.0)
              and serial_equals(0) and pending_x_keyboard.returncode == 0)
        pump(.7, joy_mode='normal')
        check('explicit_x_exit_releases_after_one_zero_without_heartbeat',
              len(keyboard) == count + 1 and serial_equals(200))
        pump(.7)

        killed_keyboard = start_keyboard('keyboard_killed', -signal.SIGKILL)
        pump(1.5)
        pump(.8, keys=b'w')
        check('replacement_keyboard_active_before_process_kill', serial_equals(300))
        last_manual_receipt = keyboard[-1][0]
        killed_at = time.monotonic()
        os.killpg(killed_keyboard.pid, signal.SIGKILL)
        killed_keyboard.wait(timeout=8)
        pump(.9)
        check('dead_manual_process_does_not_latch_control', serial_equals(240))
        resumed = next(t for t, xyz in fixture.wire_packets if t > killed_at and xyz == (240, 0, 0))
        result['metrics']['navigation_resume_after_manual_process_kill_s'] = resumed-killed_at
        result['metrics']['navigation_resume_after_last_manual_receipt_s'] = resumed-last_manual_receipt
        check('manual_process_was_intentionally_killed', killed_keyboard.returncode == -signal.SIGKILL)

        loss_at = time.monotonic()
        pump(.9, nav=None)
        check('loss_of_all_command_producers_reaches_serial_zero', serial_equals(0))
        stopped = next(t for t, xyz in fixture.wire_packets if t > loss_at and xyz == (0, 0, 0))
        result['metrics']['serial_zero_after_all_producers_stop_s'] = stopped-loss_at
        result['pass'] = True
    except Exception as error:
        result['error'] = repr(error)
        raise
    finally:
        for name, process, expected_returncode in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            returncode = process.wait(timeout=8)
            result['cleanup'].append({'name': name, 'returncode': returncode,
                'expected_returncode': expected_returncode, 'forced': False})
            result['pass'] = result['pass'] and returncode == expected_returncode
        result['observed'] = {'keyboard_commands': len(keyboard),
            'navigation_commands': len(fixture.outputs), 'requested_commands': len(fixture.requested),
            'guard_commands': len(fixture.base), 'serial_packets': len(fixture.wire_packets)}
        result['streams'] = {'keyboard': keyboard, 'requested': fixture.requested,
            'guard': fixture.base, 'serial': fixture.wire_packets}
        (args.output_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        fixture.close()
        rclpy.try_shutdown()
        for fd in (base_master, base_slave, key_master, key_slave):
            os.close(fd)
        for log in logs:
            log.close()
    assert result['pass'], result['cleanup']
    print(json.dumps({key: value for key, value in result.items() if key != 'streams'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--guard-binary', type=Path, required=True)
    parser.add_argument('--driver-binary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    run(parser.parse_args())
