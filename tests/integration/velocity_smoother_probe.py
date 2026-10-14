"""Exercise the installed smoother's last braking steps, without a robot.

Run in an isolated ROS domain. A declared 20 Hz cycle allows 0.02 m/s linear
and 0.04 rad/s angular decrements. Wall scheduling is recorded separately;
this does not establish physical actuator acceleration or a real-time bound.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
import yaml
from chassis_probe import load_profile, render


def trial(node, configuration, directory, label):
    name = 'velocity_smoother_' + label
    params = directory / (label + '.yaml')
    params.write_text(yaml.safe_dump({name: configuration}))
    log = (directory / (label + '.log')).open('w')
    process = subprocess.Popen([
        str(Path(get_package_prefix('nav2_velocity_smoother')) /
            'lib/nav2_velocity_smoother/velocity_smoother'),
        '--ros-args', '--params-file', str(params),
        '-r', '__node:=' + name,
        '-r', 'cmd_vel:=/smoother_probe/request',
        '-r', 'cmd_vel_smoothed:=/smoother_probe/output'], stdout=log, stderr=log,
        start_new_session=True)
    values = []
    subscription = node.create_subscription(Twist, '/smoother_probe/output',
        lambda m: values.append({'wall': time.monotonic(),
                                'v': [m.linear.x, m.linear.y, m.angular.z]}), 10)
    publisher = node.create_publisher(Twist, '/smoother_probe/request', 10)
    client = node.create_client(ChangeState, '/' + name + '/change_state')

    def pump(duration, message=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if message is not None:
                publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError(f'{label}: smoother exited {process.returncode}')

    try:
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Smoother lifecycle service unavailable')
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done() or not future.result().success:
                raise RuntimeError(f'{label}: lifecycle transition {transition} failed')
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            pump(.05)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('Smoother input subscriber unavailable')
        command = Twist()
        command.linear.x = command.linear.y = .0424
        command.angular.z = .0824
        pump(.6, command)
        zero_wall = time.monotonic()
        pump(.6, Twist())
        tail = [value for value in values if value['wall'] >= zero_wall - .12]
        if len(tail) < 5 or max(v['v'][0] for v in tail) < .04:
            raise RuntimeError(f'{label}: insufficient braking coverage')
        jumps = [max(abs(a['v'][i] - b['v'][i])
                     for a, b in zip(tail, tail[1:])) for i in range(3)]
        interval_max = max(b['wall'] - a['wall'] for a, b in zip(tail, tail[1:]))
        return {'label': label, 'outputs': values, 'zero_request_wall': zero_wall,
                'max_braking_step': jumps, 'max_receipt_gap_s': interval_max,
                'final_velocity': tail[-1]['v'],
                'cycle_bounds': [.02, .02, .04]}
    finally:
        if process.poll() is None:
            os.killpg(process.pid, 2)
        returncode = process.wait(timeout=5)
        log.close()
        node.destroy_subscription(subscription)
        node.destroy_publisher(publisher)
        node.destroy_client(client)
        assert returncode == 0, (label, 'smoother shutdown', returncode)


def main():
    directory = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(tempfile.mkdtemp(prefix='smoother-probe-'))
    directory.mkdir(parents=True, exist_ok=True)
    config = render(load_profile(), 'navigation')['velocity_smoother']
    current = json.loads(json.dumps(config))
    deadband_fault = json.loads(json.dumps(config))
    deadband_fault['ros__parameters']['deadband_velocity'] = [.005, .005, .003]
    rclpy.init()
    node = Node('smoother_braking_probe')
    try:
        trials = [trial(node, params, directory, label)
                  for params, label in ((deadband_fault, 'injected_deadband_fault'), (current, 'current'))]
        (directory / 'result.json').write_text(json.dumps(trials, indent=2) + '\n')
        assert any(step > bound + 1e-6 for step, bound in
                   zip(trials[0]['max_braking_step'], trials[0]['cycle_bounds'])), trials[0]
        assert all(step <= bound + 1e-6 for step, bound in
                   zip(trials[1]['max_braking_step'], trials[1]['cycle_bounds'])), trials[1]
        assert all(abs(value) < 1e-9 for value in trials[1]['final_velocity']), trials[1]
        print(json.dumps({'status': 'PASS', 'max_braking_step':
                          {x['label']: x['max_braking_step'] for x in trials}}))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
