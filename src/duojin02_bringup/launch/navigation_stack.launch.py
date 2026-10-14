"""Hardware entry for the five navigation nodes used by the current trees."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from duojin02_bringup.navigation_stack import navigation_actions
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('duojin02_bringup'),'config','nav2.yaml')),
        DeclareLaunchArgument('autostart',default_value='true'),
        DeclareLaunchArgument('log_level',default_value='info'),
        DeclareLaunchArgument('navigation_output',default_value='/cmd_vel'),
        *navigation_actions(ParameterFile(LaunchConfiguration('params_file'),allow_substs=True),
            False, ParameterValue(LaunchConfiguration('autostart'),value_type=bool),
            LaunchConfiguration('log_level'),LaunchConfiguration('navigation_output')),
    ])
