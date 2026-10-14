"""Run shipped recovery trees through stock bt_navigator and real ROS actions.

Mock planners/controllers provide deterministic outcomes; no base, sensor driver,
velocity publisher or motion hardware is launched.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose, FollowPath, NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap, IsPathValid
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml

from ros_isolation import configure_isolation

ROOT = Path(__file__).resolve().parents[2]


def pose(x, y=0.):
    value = PoseStamped()
    value.header.frame_id = 'map'
    value.header.stamp.sec = 1
    value.pose.position.x, value.pose.position.y = float(x), float(y)
    value.pose.orientation.w = 1.
    return value


def path(target, changed=False):
    value = RosPath()
    value.header = pose(0.).header
    value.poses = [pose(0.), pose(target / 2., .2 if changed else 0.), pose(target)]
    return value


def wait(future, timeout=5.):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(.005)
    assert future.done(), 'ROS future did not complete within probe budget'
    return future.result()


class Fixture(Node):
    def __init__(self, tree_directory):
        super().__init__('navigation_recovery_fixture')
        self.tree_directory = tree_directory
        self.case, self.cases, self.robot_x = None, [], 0.
        group = ReentrantCallbackGroup()
        self.servers = [ActionServer(self, action, name, execute_callback=callback,
            goal_callback=lambda request: GoalResponse.ACCEPT,
            cancel_callback=self.cancel, callback_group=group)
            for action, name, callback in [(ComputePathToPose, 'compute_path_to_pose', self.plan),
                (ComputePathThroughPoses, 'compute_path_through_poses', self.plan),
                (FollowPath, 'follow_path', self.follow)]]
        self.group, self.service = group, None
        self.make_service()
        self.clear_services = [self.create_service(ClearEntireCostmap,
            name + '/clear_entirely_' + name.split('/')[0], self.clear, callback_group=group)
            for name in ('global_costmap', 'local_costmap')]
        self.static, self.dynamic = StaticTransformBroadcaster(self), TransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
        transform.transform.rotation.w = 1.
        self.static.sendTransform(transform)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(.02, self.feedback)
        self.navigation_clients = {'pose': ActionClient(self, NavigateToPose, 'navigate_to_pose'),
                        'through': ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')}
        self.lifecycle = self.create_client(ChangeState, '/bt_navigator/change_state')
        self.path_availability = self.create_client(IsPathValid, '/is_path_valid')

    def feedback(self):
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, 'odom', 'base_footprint'
        transform.transform.translation.x, transform.transform.rotation.w = self.robot_x, 1.
        self.dynamic.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.orientation.w = self.robot_x, 1.
        self.odom.publish(odom)

    def make_service(self):
        self.service = self.create_service(IsPathValid, '/is_path_valid', self.validate, callback_group=self.group)

    def cancel(self, goal):
        self.case['cancel_requests'] += 1
        return CancelResponse.ACCEPT

    def clear(self, request, response):
        self.case['clear_requests'] += 1
        return response

    def validate(self, request, response):
        case = self.case
        case['queries'].append(request.path.poses[-1].pose.position.x)
        case['query_pending'].set()
        if case['mode'] == 'timeout':
            time.sleep(.45)
        if case['mode'] == 'preempt_query' and request.path.poses[-1].pose.position.x == 3.:
            case['release'].wait(3.)
        response.is_valid = case['mode'] == 'valid'
        case['query_finished'].set()
        return response

    def plan(self, goal):
        case = self.case
        request = goal.request
        targets = [p.pose.position.x for p in request.goals] if hasattr(request, 'goals') else [request.goal.pose.position.x]
        case['plans'].append(targets)
        if targets[-1] == 7.:
            case['fresh_plan_started'].set()
        result_type = ComputePathThroughPoses.Result if hasattr(request, 'goals') else ComputePathToPose.Result
        result = result_type()
        if case['mode'] == 'planner_failure':
            goal.abort()
            return result
        old_reply = case['mode'] == 'preempt_replan' and len(case['plans']) == 2 and targets[-1] == 3.
        if old_reply:
            case['replan_pending'].set()
            case['release'].wait(3.)
        if goal.is_cancel_requested:
            goal.canceled()
            if old_reply:
                case['replan_finished'].set()
            return result
        changed = len(case['plans']) > 1 and case['mode'] not in ('unchanged', 'trim')
        result.path = path(targets[-1], changed)
        if case['mode'] == 'unchanged' and len(case['plans']) > 1:
            result.path.header.stamp.sec += 1
        goal.succeed()
        if old_reply:
            case['replan_finished'].set()
        return result

    def follow(self, goal):
        case = self.case
        target = goal.request.path.poses[-1].pose.position.x
        case['follows'].append({'target': target, 'changed': goal.request.path.poses[1].pose.position.y != 0.})
        result = FollowPath.Result()
        if case['mode'] == 'trim':
            self.robot_x = 1.
            deadline = time.monotonic() + 5.
            while len(case['plans']) < 2 and time.monotonic() < deadline and not goal.is_cancel_requested:
                time.sleep(.01)
            assert len(case['plans']) >= 2, 'through-poses periodic planning did not run'
        else:
            time.sleep(.03)
        if goal.is_cancel_requested:
            goal.canceled()
        elif case['mode'] == 'trim' or target == 7. or (len(case['follows']) > 1 and case['mode'] != 'second_failure'):
            goal.succeed()
        else:
            goal.abort()
        return result

    def transition(self, identifier):
        request = ChangeState.Request()
        request.transition.id = identifier
        return wait(self.lifecycle.call_async(request)).success

    def send(self, route, target=3.):
        action = NavigateToPose if route == 'pose' else NavigateThroughPoses
        request = action.Goal()
        if route == 'pose':
            request.pose = pose(target)
        else:
            request.poses = [pose(1.), pose(target)]
        filename = 'navigate_to_pose.xml' if route == 'pose' else 'navigate_through_poses.xml'
        request.behavior_tree = str(self.tree_directory / filename)
        handle = wait(self.navigation_clients[route].send_goal_async(request))
        assert handle.accepted, 'navigator rejected probe goal'
        return handle.get_result_async()

    def exercise(self, route, mode):
        case = {'route': route, 'mode': mode, 'plans': [], 'follows': [], 'queries': [], 'cancel_requests': 0, 'clear_requests': 0,
                'release': threading.Event(), 'query_pending': threading.Event(), 'replan_pending': threading.Event(),
                'query_finished': threading.Event(), 'replan_finished': threading.Event(), 'fresh_plan_started': threading.Event()}
        self.case = case
        self.cases.append(case)
        self.robot_x = 0.
        if mode == 'missing':
            self.destroy_service(self.service)
            self.service = None
            deadline = time.monotonic() + 3.
            while self.path_availability.service_is_ready() and time.monotonic() < deadline:
                time.sleep(.02)
            assert not self.path_availability.service_is_ready(), 'missing service still advertised'
            case['service_absence_confirmed'] = True
            time.sleep(.2)
        first = self.send(route)
        if mode.startswith('preempt_'):
            trigger = case['query_pending'] if mode == 'preempt_query' else case['replan_pending']
            assert trigger.wait(3.), 'requested preemption window was not reached'
            replacement = self.send(route, 7.)
            assert case['fresh_plan_started'].wait(3.), 'new target did not reach planning while old reply was pending'
            outcome = wait(replacement, 7.)
            # Release old work only after the fresh target has finished. Its
            # late response must never reach another controller attempt.
            case['release'].set()
            completed = case['query_finished'] if mode == 'preempt_query' else case['replan_finished']
            assert completed.wait(2.), 'old reply did not finish after release'
            case['late_old_reply_completed_after_new_goal'] = True
            previous = wait(first, 3.)
            case['previous_status'] = previous.status
            assert previous.status != GoalStatus.STATUS_SUCCEEDED
            assert outcome.status == GoalStatus.STATUS_SUCCEEDED
            assert [entry['target'] for entry in case['follows']] == [3., 7.], case
            assert case['plans'][-1][-1] == 7., case
            time.sleep(.1)  # Late old query reply must not initiate another controller attempt.
            assert len(case['follows']) == 2, case
        else:
            outcome = wait(first, 7.)
            succeeds = mode in ('changed', 'trim')
            assert outcome.status == (GoalStatus.STATUS_SUCCEEDED if succeeds else GoalStatus.STATUS_ABORTED), case
            expected_plans = 2 if mode in ('changed', 'unchanged', 'second_failure', 'trim') else 1
            expected_follows = 0 if mode == 'planner_failure' else (2 if mode in ('changed', 'second_failure') else 1)
            assert len(case['plans']) == expected_plans and len(case['follows']) == expected_follows, case
            assert len(case['queries']) == (0 if mode in ('planner_failure', 'missing', 'trim') else 1), case
            if mode == 'trim':
                assert case['plans'] == [[1., 3.], [3.]] and case['cancel_requests'] == 0, case
        case['status'] = outcome.status
        assert case['clear_requests'] == 0, 'navigation invoked a blind costmap clear'
        if mode == 'missing':
            self.make_service()
            time.sleep(.2)
        time.sleep(.05)


def run(args):
    domain = configure_isolation()
    args.output_dir.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    fixture = Fixture(args.tree_directory)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(fixture)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    config = yaml.safe_load(args.params_file.read_text())['bt_navigator']
    parameters = config['ros__parameters']
    parameters.update(default_nav_to_pose_bt_xml=str(args.tree_directory / 'navigate_to_pose.xml'),
        default_nav_through_poses_bt_xml=str(args.tree_directory / 'navigate_through_poses.xml'),
        odom_topic='/odom', bond_heartbeat_period=0.)
    params = args.output_dir / 'navigator.yaml'
    params.write_text(yaml.safe_dump({'bt_navigator': config}))
    log = (args.output_dir / 'navigator.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'nav2_bt_navigator', 'bt_navigator', '--ros-args', '--params-file', str(params)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    result = {'pass': False, 'ros_domain_id': domain, 'scope': 'stock bt_navigator lifecycle + shipped XML + real DDS/actions; deterministic mocks, no hardware',
              'fixture_server_timeout_ms': parameters['default_server_timeout'],
              'fixture_bt_loop_ms': parameters['bt_loop_duration'],
              'fixture_wait_for_service_timeout_ms': parameters['wait_for_service_timeout'], 'cleanup': []}
    try:
        assert fixture.lifecycle.wait_for_service(timeout_sec=8), 'navigator lifecycle service unavailable'
        assert fixture.transition(1) and fixture.transition(3), 'navigator configure/activate failed; inspect navigator.log'
        for client in fixture.navigation_clients.values():
            assert client.wait_for_server(timeout_sec=3)
        for route in ('pose', 'through'):
            for mode in ('changed', 'valid', 'missing', 'timeout', 'unchanged', 'second_failure', 'planner_failure', 'preempt_query', 'preempt_replan'):
                fixture.exercise(route, mode)
        fixture.exercise('through', 'trim')
        result['pass'] = True
    except Exception as error:
        result['error'] = str(error)
    finally:
        for case in fixture.cases:
            case['release'].set()
        if process.poll() is None:
            for identifier, name in [(4, 'deactivate'), (2, 'cleanup')]:
                try:
                    result['cleanup'].append({name: fixture.transition(identifier)})
                except Exception as error:
                    result['cleanup'].append({name: str(error)})
            try:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    result['cleanup'].append({'forced_termination': True})
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=3)
            except ProcessLookupError:
                process.wait(timeout=3)
        result['cleanup'].append({'navigator_exit': process.returncode})
        result['cleanup'].append({'executor_shutdown': executor.shutdown(timeout_sec=5)})
        worker.join(timeout=2)
        result['cleanup'].append({'worker_stopped': not worker.is_alive()})
        fixture.destroy_node()
        rclpy.try_shutdown()
        log.close()
        expected = [{'deactivate': True}, {'cleanup': True}, {'navigator_exit': 0},
                    {'executor_shutdown': True}, {'worker_stopped': True}]
        result['pass'] = result['pass'] and result['cleanup'] == expected
        result['cases'] = [{key: value for key, value in case.items() if not isinstance(value, threading.Event)} for case in fixture.cases]
        (args.output_dir / 'validation.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    assert result['pass'], result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--params-file', type=Path, default=ROOT / 'src/duojin02_bringup/config/nav2.yaml')
    parser.add_argument('--tree-directory', type=Path, default=ROOT / 'src/duojin02_bringup/behavior_trees')
    run(parser.parse_args())
