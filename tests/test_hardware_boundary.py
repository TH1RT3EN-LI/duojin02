"""Contracts keeping the hardware branch independent of simulation."""

import ast
import importlib.util
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / "src/duojin01_bringup"
sys.path.insert(0, str(BRINGUP))
FORBIDDEN = re.compile(
    r"gazebo|ignition|ros_gz|orbbec|depth_cam|duojin01_(?:camera|mission|msgs)|lslidar_msgs|duojin01_(?:sim|controller_emulator|gz|safety_watchdog)")


def load_launch(name):
    path = BRINGUP / "launch" / f"{name}.launch.py"
    spec = importlib.util.spec_from_file_location(f"hardware_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_hardware_package_graph_has_no_simulation_dependencies():
    packages = {}
    for manifest in (ROOT / "src").rglob("package.xml"):
        tree = ET.parse(manifest).getroot()
        name = tree.findtext("name")
        assert not FORBIDDEN.search(name), name
        packages[name] = tree
    assert set(packages) == {
        "duojin01_base_driver", "duojin01_bringup", "duojin01_description",
        "duojin01_slam_tools", "duojin01_teleop", "duojin01_navigation", "lslidar_driver"}
    for name, tree in packages.items():
        for element in tree:
            if element.tag.endswith("depend"):
                dependency = (element.text or "").strip()
                assert not FORBIDDEN.search(dependency), (name, dependency)
                if dependency.startswith("duojin01_"):
                    assert dependency in packages, (name, dependency)


def test_no_scene_bridge_or_simulation_assets_remain():
    for path in (ROOT / "src").rglob("*"):
        assert not path.name.startswith("sim_"), path
        assert path.suffix != ".sdf", path
        assert not FORBIDDEN.search(path.name), path
    for path in (ROOT / "src/duojin01_description/urdf").rglob("*.xacro"):
        assert "<gazebo" not in path.read_text(), path


def test_all_hardware_configuration_clocks_are_real():
    def check(value, path):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "use_sim_time":
                    assert item is False, path
                check(item, path)
        elif isinstance(value, list):
            for item in value:
                check(item, path)
    for path in (BRINGUP / "config").rglob("*.yaml"):
        check(yaml.safe_load(path.read_text()), path)


@pytest.mark.parametrize("name", [
    "base", "mapping", "navigation", "nav2", "slam",
    "save_map", "joy_teleop", "online_navigation", "velocity_guard",
])
def test_launches_resolve_without_simulation_clock_arguments(name, monkeypatch):
    from launch.actions import DeclareLaunchArgument
    monkeypatch.setenv("USE_SIM_TIME", "true")
    description = load_launch(name).generate_launch_description()
    arguments = [action.name for action in description.entities
                 if isinstance(action, DeclareLaunchArgument)]
    assert "use_sim_time" not in arguments
    tree = ast.parse((BRINGUP / "launch" / f"{name}.launch.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if isinstance(key, ast.Constant) and key.value == "use_sim_time":
                    assert isinstance(value, ast.Constant)
                    assert value.value is False or value.value == "false"


def test_nav2_uses_real_time_without_synthetic_initial_pose(tmp_path, monkeypatch):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    monkeypatch.setenv("USE_SIM_TIME", "true")
    map_file = tmp_path / "0.yaml"
    map_file.write_text("image: 0.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    (tmp_path / "0.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    context = LaunchContext()
    context.launch_configurations["map"] = str(map_file)
    actions = load_launch("nav2")._create_nav_actions(
        context, get_package_share_directory("nav2_bringup"))
    include = next(action for action in actions
                   if isinstance(action, IncludeLaunchDescription))
    assert perform_substitutions(
        context, normalize_to_list_of_substitutions(
            dict(include.launch_arguments)["use_sim_time"])) == "false"
    assert "initial_pose_publisher" not in (
        BRINGUP / "launch/nav2.launch.py").read_text()


def test_command_sources_reach_base_through_validated_velocity_arbitration():
    config = yaml.safe_load((BRINGUP / "config/twist_mux.yaml").read_text())
    parameters = config["twist_mux"]["ros__parameters"]
    assert {entry["topic"] for entry in parameters["topics"].values()} == {
        "/cmd_vel", "/cmd_vel_normal", "/cmd_vel_slow"}
    assert not parameters.get("locks")
    nodes = {}
    tree = ast.parse((BRINGUP / "launch/base.launch.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "Node":
            fields = {item.arg: item.value for item in node.keywords}
            package = ast.literal_eval(fields["package"])
            if "remappings" in fields:
                nodes[package] = dict(ast.literal_eval(fields["remappings"]))
    output_topic = nodes["twist_mux"]["/cmd_vel_out"]
    assert output_topic == "/cmd_vel_requested"
    guard = yaml.safe_load((BRINGUP / "config/velocity_guard.yaml").read_text())
    monitor = guard["collision_monitor"]["ros__parameters"]
    assert monitor["cmd_vel_in_topic"] == "/cmd_vel_guarded"
    assert monitor["cmd_vel_out_topic"] == nodes["duojin01_base_driver"]["/cmd_vel"]
    assert output_topic not in {entry["topic"] for entry in parameters["topics"].values()}


def test_removed_modules_leave_no_launch_or_configuration_references():
    removed = re.compile(r"orbbec|depth_cam|watchdog|cmd_vel_safe|cmd_vel_estop|teleop_estop|duojin01_camera|duojin01_mission|gcode|calibration_file|lidar_model")
    for directory in (BRINGUP / "launch", BRINGUP / "config"):
        for path in directory.rglob("*"):
            if path.is_file() and path.suffix in {".py", ".yaml", ".rviz", ".json"}:
                assert not removed.search(path.read_text()), path


def test_generated_urdf_contains_only_hardware_geometry():
    import xacro
    document = xacro.process_file(
        str(ROOT / "src/duojin01_description/urdf/duojin01.xacro"))
    robot = ET.fromstring(document.toxml())
    assert not robot.findall(".//gazebo")
    assert not robot.findall(".//plugin")
    links = {link.attrib["name"] for link in robot.findall("link")}
    assert {"base_footprint", "base_link", "laser", "imu_link"} <= links
    assert not any("depth_cam" in name for name in links)
    for mesh in robot.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        assert filename.startswith("package://duojin01_description/")
        relative = filename.removeprefix("package://duojin01_description/")
        assert (ROOT / "src/duojin01_description" / relative).is_file(), filename


def test_lidar_upgrade_preserves_hardware_interfaces_and_scopes_parameters(monkeypatch):
    from launch import LaunchContext
    from launch.actions import GroupAction, IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    module = load_launch("base")
    lidar_share = ROOT / "src/lslidar_driver/lslidar_driver"
    shares = {"lslidar_driver": lidar_share,
              "duojin01_description": ROOT / "src/duojin01_description",
              "duojin01_bringup": BRINGUP}
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(shares[name]))
    description = module.generate_launch_description()
    groups = [item for item in description.entities if isinstance(item, GroupAction)]
    assert len(groups) == 1
    children = groups[0].get_sub_entities()
    include = next(item for item in children if isinstance(item, IncludeLaunchDescription))
    context = LaunchContext()
    context.launch_configurations.update(
        params_file="/ws/config/nav2.yaml", lidar_serial_port="/dev/test_lidar")
    include.launch_description_source.get_launch_description(context)
    path = include.launch_description_source.location
    assert Path(path) == lidar_share / "launch/lslidar_x10_launch.py"
    values = {name: perform_substitutions(context, normalize_to_list_of_substitutions(value))
              for name, value in include.launch_arguments}
    assert values["params_file"] == str(lidar_share / "config/duojin01_n10plus.yaml")
    assert values["serial_port"] == "/dev/test_lidar"
    assert values["frame_id"] == "laser"
    assert values["scan_topic"] == "/scan"
    # Exercise the group's configuration stack without executing any node actions.
    for action in groups[0].execute(context):
        if action is include:
            context.launch_configurations["params_file"] = values["params_file"]
        else:
            action.execute(context)
    assert context.launch_configurations["params_file"] == "/ws/config/nav2.yaml"
    configuration = yaml.safe_load(Path(values["params_file"]).read_text())["lslidar_driver_node"]["ros__parameters"]
    assert configuration["use_sim_time"] is False
    assert not (lidar_share / "src/lslidar_driver.cc").exists()
    assert not (lidar_share / "params/lsx10.yaml").exists()


def test_container_and_native_workspace_use_persistent_hardware_maps(tmp_path, monkeypatch):
    from duojin01_bringup import map_paths
    monkeypatch.setenv("DUOJIN01_WORKSPACE_ROOT", str(tmp_path))
    (tmp_path / "maps").mkdir()
    (tmp_path / "maps/test.pgm").write_bytes(b"P5\n1 1\n255\n\xff")
    for name in ("2", "12", "5"):
        (tmp_path / "maps" / f"{name}.yaml").write_text("image: test.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    assert map_paths.get_workspace_root() == tmp_path
    assert map_paths.pick_latest_map_yaml() == str(tmp_path / "maps/12.yaml")
    assert map_paths.resolve_map_yaml("maps/5.yaml") == str(tmp_path / "maps/5.yaml")
    monkeypatch.setenv("DUOJIN01_WORKSPACE_ROOT", "relative/path")
    with pytest.raises(RuntimeError, match="absolute"):
        map_paths.get_workspace_root()
