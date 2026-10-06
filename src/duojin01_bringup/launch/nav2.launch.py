import os
import yaml
from duojin01_bringup.lattice import for_resolution

from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.map_paths import resolve_map_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


PACKAGE_NAME = "duojin01_bringup"


def _create_nav_actions(context, nav2_share: str):
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
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
        'config/lattice_primitives/duojin01_omni_2cm_rich_lattice.json'),resolution)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
        launch_arguments={
            "slam": "False",
            "map": map_yaml,
            "use_sim_time": "false",
            "params_file": RewrittenYaml(source_file=params_file,
                param_rewrites={"lattice_filepath": lattice_path,
                    "global_costmap.global_costmap.ros__parameters.resolution":str(resolution)}, convert_types=True),
            "autostart": autostart,
            "log_level": log_level,
        }.items(),
    )

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
        nav2_launch,
        rviz_launch,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory(PACKAGE_NAME)
    nav2_share = get_package_share_directory("nav2_bringup")
    default_rviz_config = PathJoinSubstitution([bringup_share, "config", "rviz", "navigation.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(bringup_share, "config", "nav2.yaml"),
            ),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            OpaqueFunction(function=_create_nav_actions, args=[nav2_share]),
        ]
    )
