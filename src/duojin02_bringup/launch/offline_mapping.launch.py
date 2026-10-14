"""Recorded-sensor mapping: no devices, controllers, GUI or hardware bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from duojin02_bringup.middleware import middleware
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    source = LaunchConfiguration("odom_source").perform(context)
    if source not in ("recorded", "ekf"):
        raise ValueError("odom_source must be recorded or ekf")
    params = LaunchConfiguration("slam_params_file").perform(context)
    queue_size = int(LaunchConfiguration("scan_queue_size").perform(context))
    scan_start = int(LaunchConfiguration("scan_start_time_ns").perform(context))
    if queue_size < 1:
        raise ValueError("scan_queue_size must be positive")
    nodes = [
        Node(package="duojin02_slam_tools", executable="replay_tf_filter",
             name="replay_tf_filter", output="screen",
             parameters=[{"use_sim_time": True, "odom_source": source, "scan_start_time_ns": scan_start}]),
        Node(package="slam_toolbox", executable="sync_slam_toolbox_node",
             name="slam_toolbox", output="screen", parameters=[params, {
                 "use_sim_time": True, "map_update_interval": 2.0,
                 "transform_publish_period": 0.05, "scan_queue_size": queue_size,
                 "tf_buffer_duration": 120.0, "enable_interactive_mode": False,
             }]),
    ]
    if source == "ekf":
        config = os.path.join(get_package_share_directory("duojin02_bringup"), "config", "ekf.yaml")
        nodes.append(Node(package="robot_localization", executable="ekf_node",
                          name="ekf_filter_node", output="screen",
                          parameters=[config, {"use_sim_time": True}]))
    return nodes


def generate_launch_description():
    share = get_package_share_directory("duojin02_bringup")
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("odom_source", default_value="recorded",
                              description="recorded: keep bag odom TF; ekf: rerun EKF from /odom and /imu"),
        DeclareLaunchArgument("slam_params_file", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("scan_queue_size", default_value="100"),
        DeclareLaunchArgument("scan_start_time_ns", default_value="0", description="First replay scan stamp to forward (set by offline_mapping)"),
        OpaqueFunction(function=create_nodes),
    ])
