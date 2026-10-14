"""Saved-map localization and navigation from the chassis configuration."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.chassis_profile import component_file, default_profile
from duojin01_bringup.lattice import for_resolution
from duojin01_bringup.map_paths import resolve_map_yaml
from duojin01_bringup.middleware import middleware
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


PACKAGE_NAME = "duojin01_bringup"


def create_navigation(context):
    profile = LaunchConfiguration("chassis_profile").perform(context)
    own = get_package_share_directory(PACKAGE_NAME)
    nav2_share = get_package_share_directory("nav2_bringup")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    try:
        map_yaml = resolve_map_yaml(LaunchConfiguration("map").perform(context), PACKAGE_NAME)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        raise RuntimeError(f"[nav2] {exc}") from exc
    with open(map_yaml) as stream:
        resolution = yaml.safe_load(stream)["resolution"]
    lattice_path = for_resolution(
        os.path.join(own, "config/lattice_primitives/duojin01_omni_2cm_rich_lattice.json"), resolution,
    )
    parameters = RewrittenYaml(
        source_file=component_file("navigation", profile),
        param_rewrites={
            "lattice_filepath": lattice_path,
            "global_costmap.global_costmap.ros__parameters.resolution": str(resolution),
        },
        convert_types=True,
    )
    localization = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "localization_launch.py")),
        launch_arguments={
            "map": map_yaml, "use_sim_time": "false", "params_file": parameters,
            "autostart": autostart, "log_level": log_level, "use_composition": "False",
        }.items(),
    )])
    navigation = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(own, "launch", "navigation_stack.launch.py")),
        launch_arguments={
            "chassis_profile": profile, "params_file": parameters,
            "autostart": autostart, "log_level": log_level,
            "navigation_output": "/nav/localization_requested",
        }.items(),
    )])
    manager = Node(
        package=PACKAGE_NAME, executable="localization_manager", output="screen",
        parameters=[component_file("health", profile), {"use_sim_time": False}],
    )
    actions = [localization, manager]
    if autostart.perform(context).lower() == "true":
        ready = Node(
            package=PACKAGE_NAME, executable="localization_readiness", output="screen",
            parameters=[{"require_health": True}],
        )
        actions.extend([
            RegisterEventHandler(OnProcessExit(
                target_action=ready,
                on_exit=lambda event, context: [navigation] if event.returncode == 0 else [Shutdown(reason="Localization readiness failed")],
            )),
            ready,
        ])
    else:
        actions.append(navigation)
    return [
        LogInfo(msg=f"[nav2] using map: {map_yaml}"),
        *actions,
        Node(
            package="rviz2", executable="rviz2", name="rviz2", output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": False}], condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    own = get_package_share_directory(PACKAGE_NAME)
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("rviz_config", default_value=PathJoinSubstitution([own, "config", "rviz", "navigation.rviz"])),
        OpaqueFunction(function=create_navigation),
    ])
