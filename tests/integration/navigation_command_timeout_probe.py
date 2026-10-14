"""Measure raw navigation producer loss through the installed command chain.

The scan/map fixture remains healthy during the fault. The actual smoother,
localization owner, mux, guard and PTY driver run in an isolated ROS domain.
Travel is an integral of commanded speed, not physical robot displacement.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import tty

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import rclpy
import yaml

from localization_manager_probe import ROOT, SensorFixture
from chassis_probe import component_file, load_profile, render


def run(args):
    args.output_dir.mkdir(parents=True, exist_ok=False)
    configuration = render(load_profile(args.chassis_profile), 'navigation')['velocity_smoother']
    timeout = configuration['ros__parameters']['velocity_timeout']
    params = args.output_dir/'smoother.yaml'
    params.write_text(yaml.safe_dump({'velocity_smoother': configuration}))
    rclpy.init()
    fixture = SensorFixture()
    fixture.destroy_publisher(fixture.command_pub)
    fixture.command_pub = fixture.create_publisher(Twist, '/cmd_vel_nav', 10)
    raw, smooth = [], []
    fixture.create_subscription(Twist, '/cmd_vel_nav',
        lambda m: raw.append((time.monotonic(), m.linear.x)), 10)
    fixture.create_subscription(Twist, '/nav/localization_requested',
        lambda m: smooth.append((time.monotonic(), m.linear.x)), 10)
    processes, logs = [], []
    result = {'pass': False, 'velocity_timeout_s': timeout, 'checks': [], 'cleanup': [],
              'scope': 'Actual DDS command chain, controlled healthy sensors, PTY serial; no physical displacement'}

    def start(name, command):
        log = (args.output_dir/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        processes.append((name, process))
        return process

    def check(name, condition):
        if not condition:
            raise AssertionError({'case': name, 'metrics': result.get('metrics'),
                                  'health': fixture.status[-1:],
                                  'guard': fixture.guard_status[-1:],
                                  'requested': fixture.requested[-3:]})
        result['checks'].append(name)

    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    fixture.master = master
    try:
        start('manager', ['ros2', 'run', 'duojin01_bringup', 'localization_manager',
            '--ros-args', '--params-file', component_file('health', args.chassis_profile),
            '-p', 'use_sim_time:=true'])
        start('mux', ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args',
            '--params-file', component_file('twist_mux', args.chassis_profile),
            '-r', '/cmd_vel_out:=/cmd_vel_requested'])
        start('guard', [str(args.guard_binary), '--ros-args', '--params-file',
            component_file('guard', args.chassis_profile), '-p', 'use_sim_time:=true'])
        start('driver', [str(args.driver_binary), '--ros-args', '-p',
            f'usart_port_name:={os.ttyname(slave)}', '-p', 'serial_baud_rate:=57600',
            '-p', 'use_sim_time:=true', '-r', '/cmd_vel:=/cmd_vel_base'])
        start('smoother', [str(Path(get_package_prefix('nav2_velocity_smoother')) /
            'lib/nav2_velocity_smoother/velocity_smoother'), '--ros-args', '--params-file', str(params),
            '-p', 'use_sim_time:=true', '-r', 'cmd_vel:=/cmd_vel_nav',
            '-r', 'cmd_vel_smoothed:=/nav/localization_requested'])
        client = fixture.create_client(ChangeState, '/velocity_smoother/change_state')
        check('smoother_service', client.wait_for_service(timeout_sec=10))
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(fixture, future, timeout_sec=10)
            check(f'lifecycle_{transition}', future.done() and future.result().success)
        fixture.pump(3.0, nav=.24)
        check('healthy_localization', bool(fixture.status) and fixture.status[-1][1]['healthy'])
        check('steady_command_at_serial', fixture.wire_packets[-1][1] == (240, 0, 0))
        last_input_wall = raw[-1][0]
        loss_wall = time.monotonic()
        fixture.pump(2.4)
        tail = [(t, x) for t, x in smooth if t >= last_input_wall]
        brake_wall = next(t for t, x in tail if x < .24-1e-6)
        zero_wall = next(t for t, x in tail if abs(x) < 1e-9)
        serial_zero_wall = next(t for t, xyz in fixture.wire_packets
                                if t > loss_wall and xyz == (0, 0, 0))
        previous = next(value for value in reversed(smooth) if value[0] < last_input_wall)
        integral_window = [previous]+tail
        travel = sum(max(0.0, min(b[0], zero_wall)-max(a[0], last_input_wall))*a[1]
                     for a, b in zip(integral_window, integral_window[1:]))
        steady = [(t, x) for t, x in smooth if loss_wall-1 <= t < loss_wall]
        steps = [abs(a[1]-b[1]) for a, b in zip(tail, tail[1:])]
        result['metrics'] = {
            'last_raw_receipt_wall': last_input_wall, 'producer_stopped_wall': loss_wall,
            'brake_onset_s': brake_wall-last_input_wall,
            'smoother_zero_s': zero_wall-last_input_wall,
            'serial_zero_s': serial_zero_wall-last_input_wall,
            'commanded_travel_after_loss_m': travel,
            'max_braking_step_mps': max(steps), 'steady_outputs_last_second': len(steady),
            'max_steady_receipt_gap_s': max(b[0]-a[0] for a, b in zip(steady, steady[1:]))}
        check('healthy_sensors_during_producer_loss', fixture.status[-1][1]['healthy'])
        check('steady_20hz_output', 18 <= len(steady) <= 22)
        check('continuous_deceleration', max(steps) <= .02+1e-6)
        check('timeout_onset', timeout-.03 <= brake_wall-last_input_wall <= timeout+.11)
        check('serial_stop_after_deceleration', serial_zero_wall-last_input_wall <= timeout+.80)
        check('serial_stays_stopped', fixture.wire_packets[-1][1] == (0, 0, 0))
        check('onset_within_360ms', brake_wall-last_input_wall <= .36)
        check('serial_zero_within_1050ms', serial_zero_wall-last_input_wall <= 1.05)
        fixture.pump(.7, manual=.12)
        check('idle_navigation_releases_mux', fixture.wire_packets[-1][1] == (120, 0, 0))
        fixture.pump(1.0, nav=.24)
        check('fresh_producer_can_resume', fixture.wire_packets[-1][1] == (240, 0, 0))
        fixture.pump(.9, nav=0.0)
        check('explicit_zero_stops_serial', fixture.wire_packets[-1][1] == (0, 0, 0))
        result['pass'] = True
    except Exception as error:
        result['error'] = repr(error)
        raise
    finally:
        for name, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            returncode = process.wait(timeout=8)
            result['cleanup'].append({'name': name, 'returncode': returncode, 'forced': False})
            result['pass'] = result['pass'] and returncode == 0
        result['streams'] = {'raw': raw, 'smoothed': smooth, 'manager': fixture.outputs,
                             'guard': fixture.base, 'serial': fixture.wire_packets}
        result['published'] = {'clock': fixture.clock_count, 'scans': fixture.scan_count,
                               'raw_commands': fixture.command_count}
        result['guard_status'] = fixture.guard_status
        (args.output_dir/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        fixture.close()
        rclpy.try_shutdown()
        os.close(master)
        os.close(slave)
        for log in logs:
            log.close()
    assert result['pass'], result['cleanup']
    print(json.dumps({k: v for k, v in result.items() if k != 'streams'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--chassis-profile', type=Path, default=None)
    parser.add_argument('--guard-binary', type=Path, required=True)
    parser.add_argument('--driver-binary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    run(parser.parse_args())
