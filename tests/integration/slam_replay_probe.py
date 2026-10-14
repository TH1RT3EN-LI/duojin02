"""End-to-end synthetic sensor bag check; measures pipeline behavior, not mapping accuracy."""
import json
import math
import os
import signal
import subprocess
import sys
from pathlib import Path

import rosbag2_py
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage
from duojin02_slam_tools.snapshots import validate_map


def timestamp(seconds):
    ns = int(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def transform(parent, child, seconds, yaw=0):
    result = TransformStamped()
    result.header.frame_id = parent
    result.child_frame_id = child
    result.header.stamp = timestamp(seconds)
    result.transform.rotation.z = math.sin(yaw / 2)
    result.transform.rotation.w = math.cos(yaw / 2)
    return result


def make_bag(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    for topic, typename in {'/scan': 'sensor_msgs/msg/LaserScan', '/tf': 'tf2_msgs/msg/TFMessage',
                            '/tf_static': 'tf2_msgs/msg/TFMessage', '/odom': 'nav_msgs/msg/Odometry',
                            '/imu': 'sensor_msgs/msg/Imu'}.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typename, serialization_format='cdr'))
    def write(topic, message, seconds):
        writer.write(topic, serialize_message(message), int(seconds * 1e9))
    write('/tf_static', TFMessage(transforms=[transform('base_footprint', 'laser', 0),
                                             transform('base_footprint', 'imu_link', 0)]), 10)
    for index in range(27):
        seconds = 10 + index * 0.1
        yaw = index * 0.12
        stale_map = transform('map', 'odom', seconds)
        stale_map.transform.translation.x = 100.0
        write('/tf', TFMessage(transforms=[stale_map, transform('odom', 'base_footprint', seconds, yaw)]), seconds)
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.header.stamp = timestamp(seconds)
        odom.twist.twist.angular.z = 1.2
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        for axis, value in enumerate([0.001, 0.001, 1e6, 1e6, 1e6, 0.01]):
            odom.twist.covariance[axis * 7] = value
        write('/odom', odom, seconds)
        imu = Imu()
        imu.header.frame_id = 'imu_link'
        imu.header.stamp = timestamp(seconds)
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1
        imu.angular_velocity.z = 1.2
        imu.angular_velocity_covariance[0] = imu.angular_velocity_covariance[4] = 1e6
        imu.angular_velocity_covariance[8] = 0.005
        write('/imu', imu, seconds)
        if index >= 24:
            continue  # Leave clock time for EKF and TF to reach the last scan.
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = timestamp(seconds)
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / 540
        scan.angle_max = scan.angle_min + 539 * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = 0.3, 10.0, 0.1
        for ray in range(540):
            angle = scan.angle_min + ray * scan.angle_increment + yaw
            dx, dy = math.cos(angle), math.sin(angle)
            distances = []
            if abs(dx) > 1e-9:
                distances.append((5 if dx > 0 else -3) / dx)
            if abs(dy) > 1e-9:
                distances.append((4 if dy > 0 else -2) / dy)
            scan.ranges.append(min(distances))
        write('/scan', scan, seconds)
    del writer


def reload_graph(prefix, root, launch_file=None):
    os.environ['ROS_DOMAIN_ID'] = '94'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    node = Node('pose_graph_reload_probe')
    service = node.create_client(GetMap, '/slam_toolbox/dynamic_map')
    with (root / 'reload.log').open('w') as log:
        entry = [str(launch_file)] if launch_file else ['duojin02_bringup', 'slam.launch.py']
        process = subprocess.Popen(['ros2', 'launch', *entry,
                                    f'pose_graph:={prefix}', 'start_at_dock:=true'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            assert service.wait_for_service(timeout_sec=20), 'Graph reload did not start'
            future = service.call_async(GetMap.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.done() and future.result(), 'Reloaded map service failed'
            grid = future.result().map
            assert grid.info.width > 0 and grid.info.height > 0
            assert abs(grid.info.origin.position.x) < 20, 'Old map TF leaked into new map'
            print(json.dumps({'reload_width': grid.info.width, 'reload_height': grid.info.height,
                              'resolution': grid.info.resolution, 'graph_reloaded': str(prefix)}))
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            forced = False
            try:
                returncode = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                forced = True
                os.killpg(process.pid, signal.SIGKILL)
                returncode = process.wait(timeout=2)
            node.destroy_node()
            rclpy.shutdown()
            print(json.dumps({'graph_reload_cleanup': {'returncode': returncode, 'forced': forced}}))
            assert returncode == 0 and not forced, 'Graph reload did not finish normally'


def reject_corrupt_graph(root, launch_file=None):
    prefix = root / 'corrupt-graph'
    for suffix in ('.posegraph', '.data'):
        original = root / 'maps' / ('rotation_recorded' + suffix)
        Path(str(prefix) + suffix).write_bytes(original.read_bytes()[:16])
    environment = {**os.environ, 'ROS_DOMAIN_ID': '95', 'ROS_LOCALHOST_ONLY': '1'}
    log_path = root / 'corrupt-graph.log'
    with log_path.open('w') as log:
        entry = [str(launch_file)] if launch_file else ['duojin02_bringup', 'slam.launch.py']
        process = subprocess.Popen(['ros2', 'launch', *entry,
                                    f'pose_graph:={prefix}', 'start_at_dock:=true'],
                                   env=environment, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            raise AssertionError('SLAM continued running after the requested graph failed to load')
    assert 'Failed to load requested pose graph' in log_path.read_text(), log_path.read_text()
    assert process.returncode != 0, 'SLAM launch reported success after the backend rejected a corrupt graph'
    print(json.dumps({'corrupt_graph_rejected': str(prefix), 'launch_exited': True,
                      'returncode': process.returncode, 'forced': False}))


def run(root):
    root.mkdir(parents=True, exist_ok=True)
    bag = root / 'rotation-bag'
    make_bag(bag)
    # Cross-architecture execution can be much slower than the target board.
    rate = os.environ.get('SLAM_PROBE_RATE', '1.0')
    drain_timeout = os.environ.get('SLAM_PROBE_DRAIN_TIMEOUT', '20')
    for route, domain in [('recorded', 96), ('ekf', 97)]:
        subprocess.run(['ros2', 'run', 'duojin02_slam_tools', 'offline_mapping', str(bag),
                        '--odom-source', route, '--domain-id', str(domain), '--rate', rate,
                        '--output-dir', str(root / 'maps'), '--map-name', f'rotation_{route}',
                        '--drain-timeout', drain_timeout], check=True)
        validate_map(root / 'maps' / f'rotation_{route}.yaml')
    reports = [json.loads(path.read_text()) for path in (root / 'maps').glob('offline-session-*.json')]
    assert len(reports) == 2
    for report in reports:
        assert report['clock_primed'] and report['queue']['clock_active']
        assert report['recorded_scan_count'] == 24 and report['queue']['invalid'] == 0
        assert report['queue']['received'] == report['scan_count']
        assert report['warmup_skipped_scans'] == (0 if report['odom_source'] == 'recorded' else 5)
        assert report['queue']['completed'] >= 10, report
    reload_graph(root / 'maps/rotation_recorded', root)
    reject_corrupt_graph(root)


if __name__ == '__main__':
    run(Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp/slam-replay-probe'))
