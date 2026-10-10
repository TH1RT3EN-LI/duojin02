"""Hardware: online SLAM and navigation, without a saved map or AMCL."""
import os
from pathlib import Path
import yaml
from duojin01_bringup.lattice import for_resolution
from duojin01_bringup.online_profile import OnlineParameters
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from duojin01_bringup.middleware import middleware
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown, GroupAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    own=get_package_share_directory('duojin01_bringup')
    resolution=yaml.safe_load(Path(own,'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']['resolution']
    include=lambda path,args={}:IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments=args.items())
    parameters=RewrittenYaml(source_file=OnlineParameters(LaunchConfiguration('params_file')),param_rewrites={
        'lattice_filepath':for_resolution(os.path.join(own,'config/lattice_primitives/duojin01_omni_2cm_rich_lattice.json'),resolution),
        'global_costmap.global_costmap.ros__parameters.resolution':str(resolution),
        # Global hypotheses may cross unobserved space; the local map and
        # live-scan braking guard still validate executable motion.
        'global_costmap.global_costmap.ros__parameters.track_unknown_space':'false',
        'allow_unknown':'true'},convert_types=True)
    ready=Node(package='duojin01_bringup',executable='localization_readiness',output='screen')
    navigation=GroupAction([include(os.path.join(own,'launch/navigation_stack.launch.py'),{
        'params_file':parameters,'autostart':'true'})])
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument('params_file',default_value=os.path.join(own,'config/nav2.yaml')),
        DeclareLaunchArgument('use_foxglove',default_value='true'),
        DeclareLaunchArgument('use_teleop',default_value='true'),
        DeclareLaunchArgument('base_serial_port',default_value='/dev/duojin01_controller'),
        DeclareLaunchArgument('lidar_serial_port',default_value='/dev/lslidar'),
        include(os.path.join(own,'launch/base.launch.py'),{
            name:LaunchConfiguration(name) for name in ('use_foxglove','use_teleop','base_serial_port','lidar_serial_port')}),
        include(os.path.join(own,'launch/slam.launch.py')),
        RegisterEventHandler(OnProcessExit(target_action=ready,
            on_exit=lambda event,context:[navigation] if event.returncode==0 else [Shutdown(reason='Localization readiness failed')])),ready,
    ])
