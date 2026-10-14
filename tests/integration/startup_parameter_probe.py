"""Check startup-only parameter services and real driver output with a PTY."""

import argparse
from array import array
import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import tty

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import DescribeParameters, GetParameters, ListParameters, SetParameters, SetParametersAtomically
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
import yaml

from ros_isolation import configure_isolation


def feedback(velocity):
    data = bytearray([0x7b, 0])
    data.extend(struct.pack('>3h', *(int(value * 1000) for value in velocity)))
    data.extend(struct.pack('>6h', 0, 0, 16384, 0, 0, 100))
    data.extend(struct.pack('>h', 24000))
    checksum = 0
    for byte in data:
        checksum ^= byte
    return bytes(data + bytearray([checksum, 0x7d]))


def run(args):
    domain = configure_isolation()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    rclpy.init()
    node = Node('startup_parameter_probe')
    odometry, inertial, packets = [], [], []
    node.create_subscription(Odometry, '/odom', odometry.append, 100)
    node.create_subscription(Imu, '/imu', inertial.append, 100)
    commands = node.create_publisher(Twist, '/startup_parameter_probe/cmd_vel', 10)
    processes, logs, wire = [], [], bytearray()
    velocity, next_feedback = (1.0, .4, .2), 0.0
    result = {'pass': False, 'ros_domain_id': domain, 'checks': [], 'nodes': {}, 'cleanup': [],
              'scope': 'Actual DDS parameter services, startup overrides and PTY output; no physical devices',
              'binary_sha256': {name: hashlib.sha256(path.read_bytes()).hexdigest()
                                for name, path in [('driver', args.driver_binary), ('guard', args.guard_binary)]}}
    startup = {
        'usart_port_name': os.ttyname(slave), 'serial_baud_rate': 57600,
        'odom_frame_id': 'probe_odom', 'robot_frame_id': 'probe_base', 'imu_frame_id': 'probe_imu',
        'odom_x_scale': 1.25, 'odom_y_scale': .75, 'odom_z_scale_positive': 1.1,
        'odom_z_scale_negative': .9, 'loop_hz': 100, 'command_timeout_s': .37,
        'imu_yaw_rate_bias': .01, 'imu_yaw_rate_variance': .02,
        'odom_twist_variances_moving': [.011, .022, 1e6, 1e6, 1e6, .033],
        'odom_twist_variances_stopped': [.0011, .0022, 1e6, 1e6, 1e6, .0033],
    }

    def check(name, passed, metrics=None):
        result['checks'].append({'name': name, 'pass': bool(passed), 'metrics': metrics})

    def start(name, command):
        log = (args.output_dir / f'{name}.log').open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        processes.append((name, process))

    def pump(duration):
        nonlocal next_feedback
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if any(process.poll() is not None for _, process in processes):
                raise RuntimeError('A tested node exited before cleanup')
            current = time.monotonic()
            if current >= next_feedback:
                os.write(master, feedback(velocity))
                next_feedback = current + .04
            rclpy.spin_once(node, timeout_sec=min(.01, max(0.0, deadline - current)))
            try:
                wire.extend(os.read(master, 4096))
            except BlockingIOError:
                pass
            while len(wire) >= 11:
                packet = bytes(wire[:11])
                del wire[:11]
                assert packet[0] == 0x7b and packet[-1] == 0x7d, packet
                packets.append((time.monotonic(), struct.unpack('>3h', packet[3:9])))

    def rpc(service_type, path, request):
        client = node.create_client(service_type, path)
        try:
            deadline = time.monotonic() + 8
            while not client.wait_for_service(timeout_sec=.05):
                if time.monotonic() >= deadline:
                    raise TimeoutError(path)
                pump(.02)
            future = client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                pump(.01)
            if not future.done():
                future.cancel()
                raise TimeoutError(path)
            return future.result()
        finally:
            node.destroy_client(client)

    def values(name, names):
        response = rpc(GetParameters, f'/{name}/get_parameters', GetParameters.Request(names=names))
        result = {}
        for key, value in zip(names, response.values):
            parsed = Parameter.from_parameter_msg(ParameterMessage(name=key, value=value)).value
            result[key] = list(parsed) if isinstance(parsed, array) else parsed
        return result

    def parameter_contract(name, expected):
        listed = rpc(ListParameters, f'/{name}/list_parameters', ListParameters.Request()).result.names
        names = sorted(expected)
        check(f'{name}:all_startup_parameters_present', set(names) <= set(listed), list(listed))
        before = values(name, names)
        check(f'{name}:startup_overrides', before == expected, before)
        descriptors = rpc(DescribeParameters, f'/{name}/describe_parameters',
                          DescribeParameters.Request(names=names)).descriptors
        changes, statuses = [], {}
        for key in names:
            value = before[key]
            changed = (1.5 if key == 'command_timeout_s' else value + 1 if type(value) is int
                       else value + .125 if type(value) is float else value + '_changed'
                       if isinstance(value, str) else [item + .125 for item in value])
            parameter = Parameter(key, value=changed).to_parameter_msg()
            changes.append(parameter)
            response = rpc(SetParameters, f'/{name}/set_parameters', SetParameters.Request(parameters=[parameter]))
            status = response.results[0]
            statuses[key] = {'successful': status.successful, 'reason': status.reason}
        after = values(name, names)
        atomic = rpc(SetParametersAtomically, f'/{name}/set_parameters_atomically',
                     SetParametersAtomically.Request(parameters=changes)).result
        final = values(name, names)
        read_only = {key: descriptor.read_only for key, descriptor in zip(names, descriptors)}
        result['nodes'][name] = {'listed_parameters': list(listed), 'read_only': read_only,
                               'startup_values': before, 'set_results': statuses, 'after_values': after,
                               'atomic_result': {'successful': atomic.successful, 'reason': atomic.reason},
                               'final_values': final}
        descriptors_match = len(descriptors) == len(names) and all(
            descriptor.name == key for key, descriptor in zip(names, descriptors))
        check(f'{name}:all_descriptors_read_only', descriptors_match and all(read_only.values()), read_only)
        check(f'{name}:all_runtime_sets_rejected', all(not status['successful'] for status in statuses.values()))
        check(f'{name}:runtime_set_cannot_change_reported_values', before == after)
        check(f'{name}:atomic_set_rejected_without_change', not atomic.successful and before == final)

    try:
        driver_args = [str(args.driver_binary), '--ros-args', '-r', '/cmd_vel:=/startup_parameter_probe/cmd_vel']
        for key, value in startup.items():
            driver_args.extend(['-p', f'{key}:={json.dumps(value)}'])
        start('driver', driver_args)
        start('guard', [str(args.guard_binary), '--ros-args', '--params-file', str(args.params_file),
                        '-p', 'velocity_guard:command_timeout_s:=0.37'])
        parameter_contract('duojin02_base_driver', startup)
        guard = yaml.safe_load(args.params_file.read_text())
        guard_values = {**guard.get('/**', {}).get('ros__parameters', {}),
                        **guard['velocity_guard']['ros__parameters'], 'command_timeout_s': .37}
        guard_values.pop('use_sim_time', None)
        parameter_contract('velocity_guard', guard_values)
        for label, source, expected, covariance in [
            ('positive', (1.0, .4, .2), (1.25, .3, .22), startup['odom_twist_variances_moving']),
            ('negative', (1.0, .4, -.2), (1.25, .3, -.18), startup['odom_twist_variances_moving']),
            ('stopped', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), startup['odom_twist_variances_stopped']),
        ]:
            velocity = source
            previous = len(odometry)
            pump(.25)
            assert len(odometry) > previous and inertial, 'No PTY feedback output'
            odom, imu = odometry[-1], inertial[-1]
            observed = (odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z)
            diagonal = [odom.twist.covariance[axis * 7] for axis in range(6)]
            check(f'pty:{label}_startup_scaling_and_covariance',
                  all(abs(a - b) < 1e-6 for a, b in zip(observed, expected)) and diagonal == covariance,
                  {'observed': observed, 'expected': expected, 'covariance': diagonal})
            check(f'pty:{label}_startup_frames_and_imu',
                  (odom.header.frame_id, odom.child_frame_id, imu.header.frame_id) ==
                  ('probe_odom', 'probe_base', 'probe_imu') and
                  abs(imu.angular_velocity.z - (100 * .00026644 - .01)) < 1e-6 and
                  imu.angular_velocity_covariance[8] == .02)
        command = Twist()
        command.linear.x = .3
        initial = len(packets)
        commands.publish(command)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            pump(.01)
            tail = packets[initial:]
            motion = next((stamp for stamp, value in tail if value == (300, 0, 0)), None)
            stop = next((stamp for stamp, value in tail if motion is not None and stamp > motion and value == (0, 0, 0)), None)
            if stop is not None:
                break
        elapsed = stop - motion if motion is not None and stop is not None else None
        check('pty:startup_command_timeout_remains_effective', elapsed is not None and .32 <= elapsed <= .52,
              {'startup_timeout_s': .37, 'attempted_runtime_timeout_s': 1.5, 'wire_timeout_s': elapsed})
    except Exception as error:
        result['error'] = repr(error)
    finally:
        for name, process in reversed(processes):
            forced = False
            try:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                code = process.wait(timeout=8)
            except Exception as error:
                forced = True
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                code = process.wait(timeout=2)
                result.setdefault('cleanup_errors', []).append(repr(error))
            result['cleanup'].append({'name': name, 'returncode': code, 'forced': forced})
        node.destroy_node()
        rclpy.try_shutdown()
        os.close(master)
        os.close(slave)
        for log in logs:
            log.close()
        result['pass'] = ('error' not in result and all(check['pass'] for check in result['checks']) and
                          all(item['returncode'] == 0 and not item['forced'] for item in result['cleanup']))
        (args.output_dir / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    assert result['pass'], {'failed_checks': [check['name'] for check in result['checks'] if not check['pass']],
                            'error': result.get('error'), 'cleanup': result['cleanup']}
    print(json.dumps({'pass': True, 'checks': len(result['checks']), 'cleanup': result['cleanup']}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--driver-binary', type=Path, required=True)
    parser.add_argument('--guard-binary', type=Path, required=True)
    parser.add_argument('--params-file', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    run(parser.parse_args())
