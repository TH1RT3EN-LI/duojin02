"""Hardware: online SLAM and navigation, without a saved map or AMCL."""
import os
from duojin02_bringup.online_profile import OnlineParameters
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from duojin02_bringup.middleware import middleware
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown, GroupAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    own = get_package_share_directory('duojin02_bringup')

    def include(name, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(own, 'launch', name)),
            launch_arguments=arguments.items())

    parameters = OnlineParameters(
        LaunchConfiguration('params_file'), LaunchConfiguration('slam_params_file'),
        os.path.join(own, 'config/lattice_primitives/duojin02_omni_2cm_rich_lattice.json'))
    ready = Node(package='duojin02_bringup', executable='localization_readiness', output='screen')
    navigation = GroupAction([include('navigation_stack.launch.py', {
        'params_file': parameters, 'autostart': 'true',
        'log_level': LaunchConfiguration('nav_log_level')})])
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument('params_file', default_value=os.path.join(own, 'config/nav2.yaml')),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(own, 'config/slam_toolbox.yaml')),
        DeclareLaunchArgument('pose_graph', default_value=''),
        DeclareLaunchArgument('map_start_pose', default_value=''),
        DeclareLaunchArgument('start_at_dock', default_value='false'),
        DeclareLaunchArgument('nav_log_level', default_value='info'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('base_params_file', default_value=os.path.join(own, 'config/base_driver.yaml')),
        DeclareLaunchArgument('base_serial_port', default_value='/dev/duojin02_controller'),
        DeclareLaunchArgument('base_serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/lslidar'),
        DeclareLaunchArgument('odom0', default_value='/odom'),
        DeclareLaunchArgument('imu0', default_value='/imu'),
        include('base.launch.py', {
            name: LaunchConfiguration(name) for name in (
                'use_foxglove', 'use_teleop', 'base_params_file', 'base_serial_port',
                'base_serial_baudrate', 'lidar_serial_port', 'odom0', 'imu0')}),
        include('slam.launch.py', {
            name: LaunchConfiguration(name) for name in (
                'slam_params_file', 'pose_graph', 'map_start_pose', 'start_at_dock')}),
        RegisterEventHandler(OnProcessExit(target_action=ready,
            on_exit=lambda event, context: [navigation] if event.returncode == 0
            else [Shutdown(reason='Localization readiness failed')])),
        ready,
    ])
