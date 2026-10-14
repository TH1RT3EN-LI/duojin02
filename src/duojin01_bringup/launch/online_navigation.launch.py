"""Hardware online SLAM and navigation using one chassis configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from duojin01_bringup.chassis_profile import component_file, default_profile, load_profile, render
from duojin01_bringup.lattice import for_resolution
from duojin01_bringup.middleware import middleware
from duojin01_bringup.online_profile import for_online_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def create_nodes(context):
    own = get_package_share_directory("duojin01_bringup")
    profile_path = LaunchConfiguration("chassis_profile").perform(context)
    profile = load_profile(profile_path)
    resolution = render(profile, "slam")["slam_toolbox"]["ros__parameters"]["resolution"]
    parameters = RewrittenYaml(
        source_file=for_online_file(component_file("navigation", profile_path)),
        param_rewrites={
            "lattice_filepath": for_resolution(os.path.join(own, "config/lattice_primitives/duojin01_omni_2cm_rich_lattice.json"), resolution),
            "global_costmap.global_costmap.ros__parameters.resolution": str(resolution),
        },
        convert_types=True,
    )
    navigation = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(own, "launch", "navigation_stack.launch.py")),
        launch_arguments={"chassis_profile": profile_path, "params_file": parameters, "autostart": "true"}.items(),
    )])
    ready = Node(package="duojin01_bringup", executable="localization_readiness", output="screen")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(own, "launch", "base.launch.py")),
            launch_arguments={
                "chassis_profile": profile_path,
                "use_foxglove": LaunchConfiguration("use_foxglove"),
                "use_teleop": LaunchConfiguration("use_teleop"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(own, "launch", "slam.launch.py")),
            launch_arguments={"chassis_profile": profile_path}.items(),
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=ready,
            on_exit=lambda event, context: [navigation] if event.returncode == 0 else [Shutdown(reason="Localization readiness failed")],
        )),
        ready,
    ]


def generate_launch_description():
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("chassis_profile", default_value=default_profile()),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("use_teleop", default_value="true"),
        OpaqueFunction(function=create_nodes),
    ])
