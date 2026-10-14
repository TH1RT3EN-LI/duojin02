"""Collision guard derived from the chassis configuration."""

from duojin01_bringup.chassis_profile import component_file, default_profile
from duojin01_bringup.middleware import middleware
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node(context):
    profile = LaunchConfiguration("chassis_profile").perform(context)
    return [Node(
        package="duojin01_navigation", executable="velocity_guard", name="velocity_guard",
        parameters=[component_file("guard", profile), {"use_sim_time": False}], output="screen",
    )]


def generate_launch_description():
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        OpaqueFunction(function=create_node),
    ])
