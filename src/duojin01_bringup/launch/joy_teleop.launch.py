"""Joystick controls derived from the chassis configuration."""

from duojin01_bringup.chassis_profile import component_file, default_profile
from duojin01_bringup.middleware import middleware
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_nodes(context):
    profile = LaunchConfiguration("chassis_profile").perform(context)
    return [
        Node(
            package="joy", executable="joy_node", name="joy_node",
            parameters=[component_file("joy", profile), {"use_sim_time": False}],
            remappings=[("/joy", "/joy_raw")],
        ),
        Node(
            package="duojin01_teleop", executable="joy_axis_selector_node", name="joy_axis_selector_node",
            parameters=[component_file("joy_axis_selector", profile), {"use_sim_time": False}],
        ),
        Node(
            package="teleop_twist_joy", executable="teleop_node", name="teleop_slow",
            parameters=[component_file("teleop_slow", profile), {"use_sim_time": False}],
            remappings=[("/cmd_vel", "/cmd_vel_slow")],
        ),
        Node(
            package="teleop_twist_joy", executable="teleop_node", name="teleop_normal",
            parameters=[component_file("teleop_normal", profile), {"use_sim_time": False}],
            remappings=[("/cmd_vel", "/cmd_vel_normal")],
        ),
        Node(
            package="duojin01_teleop", executable="joy_launcher_node", name="joy_launcher_node",
            parameters=[component_file("joy_launcher", profile), {"use_sim_time": False}], output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        OpaqueFunction(function=create_nodes),
    ])
