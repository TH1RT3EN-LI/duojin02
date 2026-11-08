#!/usr/bin/env python3
"""Verify the installed base launch publishes configured laser TF without hardware nodes."""
import importlib.util
import json
import math
import os
import signal
import subprocess
import sys
import time


def launch_state_publisher(angle):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription, LaunchService
    from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
    from launch_ros.actions import Node

    path = os.path.join(get_package_share_directory('duojin02_bringup'), 'launch', 'base.launch.py')
    spec = importlib.util.spec_from_file_location('base_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    node = next(x for x in entities if isinstance(x, Node)
                and x.node_package == 'robot_state_publisher'
                and x.node_executable == 'robot_state_publisher')
    yaw_arg = next(x for x in entities if isinstance(x, DeclareLaunchArgument)
                   and x.name == 'lidar_yaw_deg')
    actions = [yaw_arg, SetLaunchConfiguration('publish_robot_model', 'true')]
    if angle is not None:
        actions.append(SetLaunchConfiguration('lidar_yaw_deg', angle))
    actions.append(node)
    service = LaunchService()
    service.include_launch_description(LaunchDescription(actions))
    return service.run()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--child':
        raise SystemExit(launch_state_publisher(sys.argv[2] if len(sys.argv) > 2 else None))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['ROS_DOMAIN_ID'] = '223'
    import rclpy
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = rclpy.create_node('lidar_yaw_launch_probe')
    observed = []
    def receive(msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'laser':
                q = transform.transform.rotation
                observed.append(math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)))
    node.create_subscription(TFMessage, '/tf_static', receive,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    try:
        for angle, expected in ((None, 3.7), ('-12', -12.0), ('0', 0.0)):
            observed.clear()
            args = [sys.executable, __file__, '--child'] + ([] if angle is None else [angle])
            proc = subprocess.Popen(args, start_new_session=True)
            try:
                end = time.monotonic() + 15
                while time.monotonic() < end and not observed and proc.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.2)
                if not observed or abs(math.degrees(observed[-1]) - expected) > 1e-6:
                    raise RuntimeError(f'expected {expected} degrees; observed {observed}; exit {proc.poll()}')
                print(json.dumps({'input': angle, 'expected_deg': expected,
                                  'observed_deg': math.degrees(observed[-1])}), flush=True)
            finally:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGINT)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGTERM)
                        proc.wait(timeout=3)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
