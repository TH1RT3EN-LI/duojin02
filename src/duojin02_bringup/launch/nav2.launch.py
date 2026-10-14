import os
import yaml
from duojin02_bringup.lattice import for_resolution

from ament_index_python.packages import get_package_share_directory
from duojin02_bringup.map_paths import resolve_map_yaml
from launch import LaunchDescription
from duojin02_bringup.middleware import middleware
from duojin02_bringup.gpu_profile import GpuDefault, use_gpu_argument
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown, GroupAction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


PACKAGE_NAME = "duojin02_bringup"


def _create_nav_actions(context, nav2_share: str):
    params_file = LaunchConfiguration("params_file", default=os.path.join(
        get_package_share_directory(PACKAGE_NAME), "config", "nav2.yaml"))
    autostart = LaunchConfiguration("autostart", default="true")
    log_level = LaunchConfiguration("log_level")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    try:
        map_yaml = resolve_map_yaml(LaunchConfiguration("map").perform(context), PACKAGE_NAME)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise RuntimeError(f"[nav2] {exc}") from exc

    with open(map_yaml) as stream:
        resolution=yaml.safe_load(stream)['resolution']
    lattice_path=for_resolution(os.path.join(get_package_share_directory(PACKAGE_NAME),
        'config/lattice_primitives/duojin02_omni_2cm_rich_lattice.json'),resolution)
    rewritten=RewrittenYaml(source_file=params_file.perform(context),
        param_rewrites={"lattice_filepath": lattice_path,
            "global_costmap.global_costmap.ros__parameters.resolution":str(resolution)},convert_types=True)
    arguments={"map":map_yaml,"use_sim_time":"false","params_file":rewritten,
               "autostart":autostart,"log_level":log_level,"use_composition":"False"}
    include=lambda name:GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share,"launch",name)),launch_arguments=arguments.items())])
    localization=include("localization_launch.py")
    navigation=GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(PACKAGE_NAME),'launch','navigation_stack.launch.py')),
        launch_arguments={'params_file':rewritten,'autostart':autostart,'log_level':log_level,
                          'use_gpu':LaunchConfiguration('use_gpu', default=GpuDefault(params_file)),
                          'navigation_output':'/cmd_vel'}.items())])
    monitor_config=os.path.join(get_package_share_directory(PACKAGE_NAME),'config','localization_monitor.yaml')
    monitor=Node(package=PACKAGE_NAME,executable='localization_monitor',
        parameters=[monitor_config,{'use_sim_time':False}],output='screen')
    nav_actions=[localization,monitor]
    if autostart.perform(context).lower()=='true':
        ready=Node(package="duojin02_bringup",executable="localization_readiness",
                   output="screen")
        nav_actions += [RegisterEventHandler(OnProcessExit(target_action=ready,
            on_exit=lambda event,context:[navigation] if event.returncode==0 else [Shutdown(reason="Localization readiness failed")])),ready]
    else:nav_actions.append(navigation)

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_rviz),
    )

    return [
        LogInfo(msg=f"[nav2] using map: {map_yaml}"),
        *nav_actions,
        rviz_launch,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory(PACKAGE_NAME)
    nav2_share = get_package_share_directory("nav2_bringup")
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"])

    return LaunchDescription(
        [
            middleware(),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            use_gpu_argument(),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            OpaqueFunction(function=_create_nav_actions, args=[nav2_share]),
        ]
    )
