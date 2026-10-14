"""Hardware saved-map navigation using one chassis configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.chassis_profile import default_profile
from duojin01_bringup.middleware import middleware
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    own = get_package_share_directory("duojin01_bringup")
    profile = LaunchConfiguration("chassis_profile")
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        DeclareLaunchArgument("nav_log_level", default_value="info"),
        DeclareLaunchArgument("map", default_value=""),
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
            PythonLaunchDescriptionSource(os.path.join(own, "launch", "nav2.launch.py")),
            launch_arguments={
                "chassis_profile": profile,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "autostart": LaunchConfiguration("autostart"),
                "map": LaunchConfiguration("map"),
                "log_level": LaunchConfiguration("nav_log_level"),
            }.items(),
        ),
    ])
