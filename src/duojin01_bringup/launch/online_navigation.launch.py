"""Hardware: online SLAM and navigation, without a saved map or AMCL."""
import os
from pathlib import Path
import yaml
from duojin01_bringup.lattice import for_resolution
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    own=get_package_share_directory('duojin01_bringup')
    resolution=yaml.safe_load(Path(own,'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']['resolution']
    include=lambda path,args={}:IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments=args.items())
    parameters=RewrittenYaml(source_file=LaunchConfiguration('params_file'),param_rewrites={
        'lattice_filepath':for_resolution(os.path.join(own,'config/lattice_primitives/duojin01_omni_2cm_rich_lattice.json'),resolution),
        'global_costmap.global_costmap.ros__parameters.resolution':str(resolution),
        'allow_unknown':'true'},convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument('params_file',default_value=os.path.join(own,'config/nav2.yaml')),
        DeclareLaunchArgument('use_foxglove',default_value='true'),
        DeclareLaunchArgument('use_teleop',default_value='true'),
        DeclareLaunchArgument('base_serial_port',default_value='/dev/duojin01_controller'),
        DeclareLaunchArgument('lidar_serial_port',default_value='/dev/lslidar'),
        include(os.path.join(own,'launch/base.launch.py'),{
            name:LaunchConfiguration(name) for name in ('use_foxglove','use_teleop','base_serial_port','lidar_serial_port')}),
        include(os.path.join(own,'launch/slam.launch.py')),
        include(os.path.join(get_package_share_directory('nav2_bringup'),'launch/navigation_launch.py'),{
            'use_sim_time':'false','params_file':parameters,'autostart':'true','use_composition':'False'}),
    ])
