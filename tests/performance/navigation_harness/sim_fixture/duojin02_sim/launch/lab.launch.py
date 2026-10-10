import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import GroupAction, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml
from duojin01_bringup.lattice import for_resolution
from duojin01_bringup.online_profile import for_online_file
from duojin01_bringup.middleware import middleware
from duojin01_bringup.navigation_stack import navigation_actions
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from duojin02_sim.scene import generate


def start(context):
    desc = get_package_share_directory('duojin01_description')
    if LaunchConfiguration('description_path').perform(context):
        desc = LaunchConfiguration('description_path').perform(context)
    bringup = get_package_share_directory('duojin01_bringup')
    profile_path = LaunchConfiguration('profile').perform(context)
    profile = yaml.safe_load(Path(profile_path).read_text())
    if profile['base']['mode'] != 'kinematic':
        raise ValueError('This launch validates kinematic base mode; roller contact dynamics is not implemented.')
    nav2 = yaml.safe_load(Path(bringup, 'config/nav2.yaml').read_text())
    footprint = nav2['local_costmap']['local_costmap']['ros__parameters']['footprint']
    # Sensor offset follows the aligned CAD model, so main->sim updates propagate.
    lidar_pose = ET.parse(Path(desc, 'models/dual_arm_mobile/model.sdf')).findtext("model/link[@name='lidar_link']/pose")
    laser_xy = list(map(float, lidar_pose.split()[:2]))
    session = Path(tempfile.mkdtemp(prefix='duojin02-lab-'))
    scenario = LaunchConfiguration('scenario').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    if scenario:
        from duojin02_sim.benchmark_scene import generate_benchmark
        scene, urdf, reference_map, _ = generate_benchmark(desc, profile_path, session, scenario,
            noise=LaunchConfiguration('noise').perform(context) == 'true')
        map_file = map_file or reference_map
        print('Benchmark reference: ' + str(session/'case.json'), flush=True)
    else:
        scene, urdf = generate(desc, profile_path, session,
            noise=LaunchConfiguration('noise').perform(context) == 'true',
            observer=LaunchConfiguration('observer').perform(context) == 'true')
    bridge = []
    def mapping(ros, gz, ros_type, gz_type, direction='GZ_TO_ROS', lazy=False):
        bridge.append(dict(ros_topic_name=ros, gz_topic_name=gz, ros_type_name=ros_type,
                           gz_type_name=gz_type, direction=direction, lazy=lazy))
    mapping('/clock', '/world/duojin02_lab/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock')
    mapping('/sim/cmd_vel', '/sim/cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ')
    mapping('/sim/lidar/raw', '/sim/lidar/raw', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan')
    mapping('/sim/imu/raw', '/sim/imu/raw', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')
    mapping('/sim/ground_truth/odom', '/sim/ground_truth/odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry')
    mapping('/joint_states', '/dual_arm/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model')
    mapping('/dual_arm/joint_trajectory', '/dual_arm/joint_trajectory', 'trajectory_msgs/msg/JointTrajectory',
            'gz.msgs.JointTrajectory', 'ROS_TO_GZ')
    for name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        mapping('/sim/wheels/' + name, '/sim/wheels/' + name, 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ')
    for name in ('head', 'left', 'right'):
        topic = '/dual_arm/' + name + '_camera_link/image'
        mapping(topic, topic, 'sensor_msgs/msg/Image', 'gz.msgs.Image', lazy=True)
        mapping('/dual_arm/' + name + '_camera_link/camera_info',
                '/dual_arm/' + name + '_camera_link/camera_info',
                'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo', lazy=True)
    mapping('/sim/overview/image', '/sim/overview/image', 'sensor_msgs/msg/Image', 'gz.msgs.Image', lazy=True)
    bridge_file = session / 'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(bridge))
    # Record generated world/bridge for inspection, without changing the reviewed assets.
    print('Gazebo runtime world: ' + scene, flush=True)
    args = ['gz', 'sim', '-r', '-v', '3', scene]
    if LaunchConfiguration('headless').perform(context) == 'true':
        args += ['-s', '--headless-rendering']
    else:
        args += ['--gui-config', os.path.join(get_package_share_directory('duojin02_sim'), 'config/gui.config')]
    if LaunchConfiguration('mapping').perform(context) == 'true':
        backend = Path(get_package_share_directory('slam_toolbox'))
        if not (backend/'hardware_build.json').is_file():
            raise RuntimeError('Mapping requires the unchanged pinned hardware SLAM backend overlay.')
    guard = LaunchConfiguration('guard').perform(context) == 'true'
    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(desc, 'models')),
        ExecuteProcess(cmd=args, output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge',
             parameters=[{'config_file': str(bridge_file), 'use_sim_time': True}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf, 'use_sim_time': True}], output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             parameters=[os.path.join(bringup, 'config/twist_mux.yaml'), {'use_sim_time': True}],
             remappings=[('/cmd_vel_out', '/cmd_vel_requested' if guard else '/cmd_vel_base')], output='screen'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(bringup, 'config/ekf.yaml'), {'use_sim_time': True}], output='screen'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[os.path.join(bringup, 'config/slam_toolbox.yaml'),
                         {'use_sim_time': True, 'map_file_name': LaunchConfiguration('slam_graph').perform(context),
                          'map_start_pose': [0.0, 0.0, 0.0]}],
             condition=IfCondition(LaunchConfiguration('mapping')), output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(get_package_share_directory('duojin02_sim'), 'config/inspection.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        Node(package='duojin02_sim', executable='device_adapter',
             parameters=[dict(use_sim_time=True, **{k:v for k,v in profile['base'].items() if k != 'mode'},
                              published_range_max_m=profile['lidar']['published_range_max_m'],
                              published_range_min_m=profile['lidar']['range_min_m'],
                              self_filter=profile['lidar']['self_filter'], footprint=footprint,
                              laser_xy=laser_xy)], output='screen')]
    if guard:
        guard_config = os.path.join(bringup, 'config/velocity_guard.yaml')
        actions.append(Node(package='duojin01_navigation', executable='velocity_guard', name='velocity_guard',
                            parameters=[guard_config, {'use_sim_time': True}], output='screen'))
    if LaunchConfiguration('navigation').perform(context) == 'true':
        params = LaunchConfiguration('nav_params').perform(context) or os.path.join(bringup,'config/nav2.yaml')
        if LaunchConfiguration('mapping').perform(context) == 'true':
            params = for_online_file(params)
        map_resolution=yaml.safe_load(Path(bringup,'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']['resolution'] if LaunchConfiguration('mapping').perform(context)=='true' else yaml.safe_load(Path(map_file).read_text())['resolution']
        lattice=for_resolution(os.path.join(bringup,'config/lattice_primitives/duojin01_omni_2cm_rich_lattice.json'),map_resolution)
        rewritten = RewrittenYaml(source_file=params, param_rewrites={
            'use_sim_time': 'true',
            'global_costmap.global_costmap.ros__parameters.track_unknown_space':'false' if LaunchConfiguration('mapping').perform(context)=='true' else 'true',
            'allow_unknown':'true' if LaunchConfiguration('mapping').perform(context)=='true' else 'false',
            'lattice_filepath': lattice,
            'global_costmap.global_costmap.ros__parameters.resolution':str(map_resolution)}, convert_types=True)
        arguments={'map':map_file,'use_sim_time':'true','params_file':rewritten,
                'autostart':'true','use_composition':'False','log_level':'warn'}
        include=lambda name:GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),'launch',name)),
            launch_arguments=arguments.items())])
        online = LaunchConfiguration('mapping').perform(context) == 'true'
        navigation=GroupAction(navigation_actions(ParameterFile(rewritten,allow_substs=True), True, True, 'info',
            '/cmd_vel' if online else '/nav/localization_requested'))
        localized=Node(package='duojin01_bringup',executable='localization_readiness',
                       parameters=[{'use_sim_time':True,'timeout_s':75.0,'require_health':not online}],output='screen')
        manager=Node(package='duojin01_bringup',executable='localization_manager',
            parameters=[os.path.join(bringup,'config/localization_health.yaml'),{'use_sim_time':True}],output='screen')
        after_sensors=[localized] if online else [include('localization_launch.py'),manager,localized]
        ready=Node(package='duojin02_sim',executable='navigation_readiness',output='screen')
        actions += [RegisterEventHandler(OnProcessExit(target_action=localized,
                     on_exit=lambda event,context:[navigation] if event.returncode==0 else [Shutdown(reason='Localization readiness failed')])),
                    RegisterEventHandler(OnProcessExit(target_action=ready,
                     on_exit=lambda event,context:after_sensors if event.returncode==0 else [Shutdown(reason='Sensor readiness failed')])),ready]

    return actions


def generate_launch_description():
    own = get_package_share_directory('duojin02_sim')
    return LaunchDescription([
        middleware(),
        SetEnvironmentVariable('CYCLONEDDS_URI', EnvironmentVariable('CYCLONEDDS_URI',
            default_value='file://'+os.path.join(own,'config/cyclonedds.xml'))),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('noise', default_value='true'),
        DeclareLaunchArgument('observer', default_value='true'),
        DeclareLaunchArgument('mapping', default_value='false'),
        DeclareLaunchArgument('guard', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='false'),
        DeclareLaunchArgument('nav_params', default_value=''),
        DeclareLaunchArgument('scenario', default_value=''),
        DeclareLaunchArgument('description_path', default_value=''),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('slam_graph', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('profile', default_value=os.path.join(own, 'config/profile.yaml')),
        OpaqueFunction(function=start)])
