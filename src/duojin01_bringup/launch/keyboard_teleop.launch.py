"""Keyboard controls use the same chassis configuration as all other inputs."""
from duojin01_bringup.chassis_profile import component_file, default_profile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node(context):
    return [Node(package='duojin01_teleop', executable='keyboard_teleop_node',
                 name='keyboard_teleop_node', output='screen', emulate_tty=True,
                 parameters=[component_file('keyboard', LaunchConfiguration('chassis_profile').perform(context))])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('chassis_profile', default_value=default_profile()),
        OpaqueFunction(function=create_node),
    ])
