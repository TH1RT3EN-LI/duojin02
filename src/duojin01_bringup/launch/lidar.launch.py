"""Standalone receive-only lidar entry using the chassis configuration."""
from duojin01_bringup.chassis_profile import component_file, default_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node(context):
    return [Node(package='lslidar_driver', executable='lslidar_driver_node',
                 name='lslidar_driver_node', output='screen', emulate_tty=True,
                 parameters=[component_file('lidar', LaunchConfiguration('chassis_profile').perform(context))])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('chassis_profile', default_value=default_profile()),
        OpaqueFunction(function=create_node),
    ])
