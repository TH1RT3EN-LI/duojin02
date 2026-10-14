import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION = ROOT / "src/duojin02_description"
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


@pytest.fixture(scope="module")
def measurements():
    return json.loads((ROOT / "docs/chassis/measurements.json").read_text())


@pytest.fixture(scope="module")
def robot():
    return ET.fromstring(xacro.process_file(str(DESCRIPTION / "urdf/duojin02.xacro")).toxml())


def vertices(name):
    document = ET.parse(DESCRIPTION / "meshes" / (name + ".dae"))
    assert document.find("c:asset/c:unit", NS).attrib["meter"] == "1"
    assert document.findtext("c:asset/c:up_axis", namespaces=NS) == "Z_UP"
    values = list(
        map(
            float,
            document.findtext('.//c:source[@id="positions"]/c:float_array', namespaces=NS).split(),
        )
    )
    return list(zip(values[::3], values[1::3], values[2::3]))


def test_wheel_joints_use_hub_centers_and_true_mecanum_meshes(robot, measurements):
    mapping = {
        "upper_left": "front_left",
        "lower_left": "rear_left",
        "upper_right": "front_right",
        "lower_right": "rear_right",
    }
    assets = set()
    for prefix, name in mapping.items():
        rotation = robot.find(f"joint[@name='{prefix}_wheel_joint']")
        xyz = list(map(float, rotation.find("origin").attrib["xyz"].split()))
        assert xyz == pytest.approx(measurements["wheels"][name]["center_xyz_m"], abs=1e-9)
        assert rotation.attrib["type"] == "continuous"
        assert rotation.find("parent").attrib["link"] == "base_link"
        assert rotation.find("axis").attrib["xyz"] == "0 1 0"
        link = robot.find(f"link[@name='{prefix}_wheel']")
        mesh = link.find("visual/geometry/mesh")
        assert "scale" not in mesh.attrib
        assets.add(mesh.attrib["filename"])
        points = vertices("chassis_" + name + "_wheel")
        width = max(p[1] for p in points) - min(p[1] for p in points)
        radius = max(math.hypot(p[0], p[2]) for p in points)
        assert width == pytest.approx(0.050, abs=1e-7)
        assert 0.0503 < radius < 0.0504
        envelope = link.find("collision/geometry/cylinder")
        assert float(envelope.attrib["radius"]) >= radius - 1e-9
    assert len(assets) == 4
    centers = [w["center_xyz_m"] for w in measurements["wheels"].values()]
    assert sum(c[0] for c in centers) == pytest.approx(0, abs=1e-9)
    assert sum(c[1] for c in centers) == pytest.approx(0, abs=1e-9)
    assert 0.312 < measurements["wheelbase_m"] < 0.314
    assert 0.354 < measurements["track_width_m"] < 0.356


def test_lidar_mount_is_on_rotation_axis_and_scan_calibration_is_separate(robot, measurements):
    mount = robot.find("joint[@name='lidar_mount_joint']")
    xyz = list(map(float, mount.find("origin").attrib["xyz"].split()))
    assert xyz == pytest.approx(measurements["lidar"]["axis_datum_xyz_m"], abs=1e-9)
    assert 0.092 < xyz[0] < 0.093 and abs(xyz[1]) < 0.00002
    scan = robot.find("joint[@name='lidar_joint']")
    assert scan.find("parent").attrib["link"] == "lidar_mount"
    assert robot.find("link[@name='laser']/visual") is None
    points = vertices("n10plus")
    bounds_center_x = (min(p[0] for p in points) + max(p[0] for p in points)) / 2
    assert bounds_center_x < -0.009
    assert robot.find("link[@name='lidar_mount']/visual/geometry/mesh") is not None


def test_navigation_envelope_uses_chassis_projection(measurements):
    assert measurements["selected_navigation_projection"] == "chassis"
    config = yaml.safe_load((ROOT / "src/duojin02_bringup/config/nav2.yaml").read_text())
    polygon = measurements["projections"][measurements["selected_navigation_projection"]][
        "navigation_polygon_xy_m"
    ]
    for costmap in ["local_costmap", "global_costmap"]:
        parameters = config[costmap][costmap]["ros__parameters"]
        assert json.loads(parameters["footprint"]) == polygon

        padding = parameters["footprint_padding"]
        padded = [
            (x + math.copysign(padding, x), y + math.copysign(padding, y)) for x, y in polygon
        ]
        inscribed = min(
            abs(a[0] * b[1] - a[1] * b[0]) / math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(padded, padded[1:] + padded[:1])
        )
        assert parameters["inflation_layer"]["inflation_radius"] > inscribed
    for x, y in measurements["projections"]["chassis"]["convex_hull_xy_m"]:
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            assert (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) >= -1e-7


def test_visual_asset_hashes_and_body_collision_envelope(measurements, robot):
    for name, asset in measurements["assets"].items():
        path = DESCRIPTION / "meshes" / (name + ".dae")
        assert hashlib.sha256(path.read_bytes()).hexdigest() == asset["sha256"]
    collision = robot.find("link[@name='base_link']/collision[@name='chassis_body_envelope']")
    center = list(map(float, collision.find("origin").attrib["xyz"].split()))
    size = list(map(float, collision.find("geometry/box").attrib["size"].split()))
    for name in ["chassis_shell", "chassis_metal", "chassis_battery"]:
        for point in vertices(name):
            assert all(abs(point[i] - center[i]) <= size[i] / 2 + 1e-9 for i in range(3))
