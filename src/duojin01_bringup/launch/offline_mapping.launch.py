"""Recorded-sensor mapping using the chassis configuration's replay settings."""

from duojin01_bringup.chassis_profile import component_file, default_profile
from duojin01_bringup.middleware import middleware
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    source = LaunchConfiguration("odom_source").perform(context)
    if source not in ("recorded", "ekf"):
        raise ValueError("odom_source must be recorded or ekf")
    profile = LaunchConfiguration("chassis_profile").perform(context)
    scan_start = int(LaunchConfiguration("scan_start_time_ns").perform(context))
    nodes = [
        Node(
            package="duojin01_slam_tools", executable="replay_tf_filter", name="replay_tf_filter",
            output="screen", parameters=[{
                "use_sim_time": True, "odom_source": source, "scan_start_time_ns": scan_start,
            }],
        ),
        Node(
            package="slam_toolbox", executable="sync_slam_toolbox_node", name="slam_toolbox",
            output="screen", parameters=[component_file("slam_offline", profile), {"use_sim_time": True}],
        ),
    ]
    if source == "ekf":
        nodes.append(Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen", parameters=[component_file("ekf", profile), {"use_sim_time": True}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        DeclareLaunchArgument("odom_source", default_value="recorded", description="recorded: keep bag odom TF; ekf: rerun EKF from /odom and /imu"),
        DeclareLaunchArgument("scan_start_time_ns", default_value="0", description="First replay scan stamp to forward"),
        OpaqueFunction(function=create_nodes),
    ])
