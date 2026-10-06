"""N10Plus serial driver connected to the Duojin01 hardware interfaces."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("lslidar_driver"),
                          "config", "duojin01_n10plus.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument("serial_port", default_value="/dev/lslidar"),
        DeclareLaunchArgument("frame_id", default_value="laser"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        Node(
            package="lslidar_driver",
            executable="lslidar_driver_node",
            name="lslidar_driver_node",
            namespace="",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file"), {
                "serial_port": LaunchConfiguration("serial_port"),
                "frame_id": LaunchConfiguration("frame_id"),
                "laserscan_topic": LaunchConfiguration("scan_topic"),
                "use_sim_time": False,
            }],
        ),
    ])
