"""Online mapping and explicit continuation from a paired pose graph."""

import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from duojin01_slam_tools.snapshots import artifact_path, validate_map
from launch import LaunchDescription
from duojin01_bringup.middleware import middleware
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create_node(context):
    share = Path(get_package_share_directory("slam_toolbox"))
    if not (share / "hardware_build.json").is_file():
        raise RuntimeError("Build and source the pinned SLAM backend with ./scripts/build_hardware.sh")
    parameters = {"use_sim_time": False}
    graph = LaunchConfiguration("pose_graph").perform(context).strip()
    start = LaunchConfiguration("map_start_pose").perform(context).strip()
    dock = LaunchConfiguration("start_at_dock").perform(context).lower() == "true"
    if graph:
        prefix = Path(graph).expanduser().resolve()
        if prefix.suffix == ".posegraph":
            prefix = prefix.with_suffix("")
        for extension in (".posegraph", ".data"):
            artifact = artifact_path(prefix, extension)
            if not artifact.is_file() or artifact.stat().st_size == 0:
                raise ValueError(f"Missing pose graph artifact: {artifact}")
        raster = artifact_path(prefix, ".yaml")
        if raster.exists():
            validate_map(raster)
        if bool(start) == dock:
            raise ValueError("For continuation, specify either map_start_pose:='[x, y, yaw]' or start_at_dock:=true")
        parameters["map_file_name"] = str(prefix)
        if start:
            pose = yaml.safe_load(start)
            if not isinstance(pose, list) or len(pose) != 3 or not all(
                isinstance(value, (int, float)) and math.isfinite(value) for value in pose
            ):
                raise ValueError("map_start_pose must contain three finite values [x, y, yaw]")
            parameters["map_start_pose"] = [float(value) for value in pose]
        else:
            parameters["map_start_at_dock"] = True
    elif start or dock:
        raise ValueError("pose_graph is required when selecting a continuation pose")
    return [Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                 name="slam_toolbox", output="screen",
                 parameters=[LaunchConfiguration("slam_params_file"), parameters],
                 on_exit=[Shutdown(reason="SLAM node exited")])]


def generate_launch_description():
    share = get_package_share_directory("duojin01_bringup")
    return LaunchDescription([
        middleware(),
        DeclareLaunchArgument("slam_params_file", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("pose_graph", default_value="", description="Saved graph prefix or .posegraph file"),
        DeclareLaunchArgument("map_start_pose", default_value="", description="Explicit continuation pose [x, y, yaw]"),
        DeclareLaunchArgument("start_at_dock", default_value="false", description="Continue at the graph's first scan pose"),
        OpaqueFunction(function=create_node),
    ])
