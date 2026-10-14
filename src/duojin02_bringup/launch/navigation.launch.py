import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from duojin02_bringup.middleware import middleware
from duojin02_bringup.gpu_profile import use_gpu_argument
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("duojin02_bringup")

    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_foxglove = LaunchConfiguration("use_foxglove")
    nav_log_level = LaunchConfiguration("nav_log_level")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "base.launch.py")),
        launch_arguments={
            "use_foxglove": use_foxglove,
            "use_teleop": LaunchConfiguration("use_teleop"),
            "base_params_file": LaunchConfiguration("base_params_file"),
            "base_serial_port": LaunchConfiguration("base_serial_port"),
            "base_serial_baudrate": LaunchConfiguration("base_serial_baudrate"),
            "lidar_serial_port": LaunchConfiguration("lidar_serial_port"),
            "use_lidar": "true",
            "odom0": odom0,
            "imu0": imu0,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_rviz": use_rviz,
            "autostart": autostart,
            "map": map_yaml,
            "params_file": params_file,
            "use_gpu": LaunchConfiguration("use_gpu"),
            "log_level": nav_log_level,
        }.items(),
    )

    return LaunchDescription(
        [
            middleware(),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("use_teleop", default_value="true"),
            DeclareLaunchArgument("base_params_file", default_value=os.path.join(bringup_share, "config", "base_driver.yaml")),
            DeclareLaunchArgument("base_serial_port", default_value="/dev/duojin02_controller"),
            DeclareLaunchArgument("base_serial_baudrate", default_value="115200"),
            DeclareLaunchArgument("lidar_serial_port", default_value="/dev/lslidar"),
            DeclareLaunchArgument("nav_log_level", default_value="info"),
            DeclareLaunchArgument("odom0", default_value="/odom"),
            DeclareLaunchArgument("imu0", default_value="/imu"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            use_gpu_argument(),
            base_launch,
            nav_launch,
        ]
    )
