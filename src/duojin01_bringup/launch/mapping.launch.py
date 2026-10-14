"""Hardware mapping and graph continuation using one chassis configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.chassis_profile import default_profile
from duojin01_bringup.middleware import middleware
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    own = get_package_share_directory("duojin01_bringup")
    profile = LaunchConfiguration("chassis_profile")
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        DeclareLaunchArgument("pose_graph", default_value=""),
        DeclareLaunchArgument("map_start_pose", default_value=""),
        DeclareLaunchArgument("start_at_dock", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([own, "config", "rviz", "mapping.rviz"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(own, "launch", "base.launch.py")),
            launch_arguments={
                "chassis_profile": profile,
                "use_foxglove": LaunchConfiguration("use_foxglove"),
                "use_teleop": LaunchConfiguration("use_teleop"),
                "use_lidar": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(own, "launch", "slam.launch.py")),
            launch_arguments={
                "chassis_profile": profile,
                **{key: LaunchConfiguration(key) for key in ("pose_graph", "map_start_pose", "start_at_dock")},
            }.items(),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": False}], condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
