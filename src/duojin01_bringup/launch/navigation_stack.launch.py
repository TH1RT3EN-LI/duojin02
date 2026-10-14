"""Navigation nodes; params_file carries only internally derived ROS parameters."""

from duojin01_bringup.chassis_profile import component_file, default_profile
from duojin01_bringup.navigation_stack import navigation_actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def create_navigation(context):
    profile = LaunchConfiguration("chassis_profile").perform(context)
    parameters = LaunchConfiguration("params_file").perform(context)
    if not parameters:
        parameters = component_file("navigation", profile)
    return navigation_actions(
        ParameterFile(parameters, allow_substs=True), False,
        ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
        LaunchConfiguration("log_level"), LaunchConfiguration("navigation_output"),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        DeclareLaunchArgument("params_file", default_value="", description="Internal derived Nav2 ROS parameter document"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("navigation_output", default_value="/cmd_vel"),
        OpaqueFunction(function=create_navigation),
    ])
