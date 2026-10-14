"""Hardware sensors and arbitration derived from the chassis configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.chassis_profile import component_file, default_profile, robot_description as render_robot_description
from duojin01_bringup.middleware import middleware
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def create_nodes(context):
    profile = LaunchConfiguration("chassis_profile").perform(context)
    bringup_share = get_package_share_directory("duojin01_bringup")
    description_share = get_package_share_directory("duojin01_description")
    urdf_file = os.path.join(description_share, "urdf", "duojin01.xacro")
    robot_description = ParameterValue(render_robot_description(profile, urdf_file), value_type=str)
    return [
        Node(
            package="twist_mux", executable="twist_mux", name="twist_mux",
            output="screen", parameters=[component_file("twist_mux", profile), {"use_sim_time": False}],
            remappings=[("/cmd_vel_out", "/cmd_vel_requested")],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "velocity_guard.launch.py")),
            launch_arguments={"chassis_profile": profile}.items(),
        ),
        Node(
            package="duojin01_base_driver", executable="duojin01_base_driver_node", name="base_driver",
            output="screen", parameters=[component_file("base", profile), {"use_sim_time": False}],
            remappings=[("/cmd_vel", "/cmd_vel_base")],
        ),
        Node(
            package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher",
            output="screen", parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("publish_joint_states")),
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
            output="screen", parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("publish_robot_model")),
        ),
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen", parameters=[component_file("ekf", profile), {"use_sim_time": False}],
        ),
        Node(
            package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge_node",
            output="screen", parameters=[component_file("foxglove", profile), {"use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("use_foxglove")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "joy_teleop.launch.py")),
            launch_arguments={"chassis_profile": profile}.items(),
            condition=IfCondition(LaunchConfiguration("use_teleop")),
        ),
        Node(
            package="lslidar_driver", executable="lslidar_driver_node", name="lslidar_driver_node",
            output="screen", emulate_tty=True,
            parameters=[component_file("lidar", profile), {"use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("use_lidar")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        DeclareLaunchArgument("publish_robot_model", default_value="true"),
        DeclareLaunchArgument("publish_joint_states", default_value="true"),
        OpaqueFunction(function=create_nodes),
    ])
