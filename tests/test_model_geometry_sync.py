"""Validate staged simulator frames against CAD and preserved assembly geometry."""
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / "src/duojin01_description"
MODEL = PACKAGE / "models/dual_arm_mobile"
DATA = json.loads((ROOT / "docs/chassis/measurements.json").read_text())
ALIGNMENT = json.loads((MODEL / "hardware_alignment.json").read_text())
SDF = ET.parse(MODEL / "model.sdf").getroot().find("model")
URDF = ET.parse(PACKAGE / "urdf/dual_arm_mobile.urdf").getroot()


def values(text, count=6):
    return list(map(float, (text or " ".join(["0"] * count)).split()))


def urdf_positions():
    positions = {"base_footprint": [0., 0., 0.]}
    pending = list(URDF.findall("joint"))
    while pending:
        resolved = []
        for joint in pending:
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            if parent in positions:
                # Parent rotations are zero on every CAD body in this staged rig;
                # only leaf optical frames have nonzero rotations.
                local = values(joint.find("origin").get("xyz"), 3)
                positions[child] = [positions[parent][i] + local[i] for i in range(3)]
                resolved.append(joint)
        assert resolved, "URDF contains a disconnected link or cycle"
        pending = [joint for joint in pending if joint not in resolved]
    return positions


def test_wheel_frames_and_collision_dimensions_match_hardware():
    positions = urdf_positions()
    for name, wheel in DATA["wheels"].items():
        link_name = name + "_wheel"
        link = SDF.find(f"link[@name='{link_name}']")
        assert values(link.findtext("pose"))[:3] == pytest.approx(wheel["center_xyz_m"], abs=1e-9)
        assert positions[link_name] == pytest.approx(wheel["center_xyz_m"], abs=1e-9)
        cylinder = link.find("collision/geometry/cylinder")
        assert float(cylinder.findtext("radius")) == pytest.approx(wheel["radial_envelope_radius_m"], abs=1e-10)
        assert float(cylinder.findtext("length")) == pytest.approx(.05, abs=1e-7)
        assert values(link.findtext("collision/pose"))[:3] == [0., 0., 0.]
    drive = SDF.find("plugin[@name='gz::sim::systems::MecanumDrive']")
    assert float(drive.findtext("wheelbase")) == pytest.approx(DATA["wheelbase_m"], abs=1e-9)
    assert float(drive.findtext("wheel_separation")) == pytest.approx(DATA["track_width_m"], abs=1e-9)
    bounds = DATA["chassis_body_collision_bounds_m"]
    body = SDF.find("link[@name='base_link']/collision")
    assert values(body.findtext("pose"))[:3] == pytest.approx([(a + b) / 2 for a, b in zip(*bounds)], abs=1e-9)
    assert values(body.findtext("geometry/box/size"), 3) == pytest.approx([b - a for a, b in zip(*bounds)], abs=1e-9)


def test_laser_frame_matches_hardware_without_claiming_optical_calibration():
    assert urdf_positions()["laser"] == pytest.approx(DATA["lidar"]["nominal_scan_xyz_m"], abs=1e-9)
    frame = SDF.find("frame[@name='laser']")
    assert frame.get("attached_to") == "lidar_link"
    assert frame.find("pose").get("relative_to") == "lidar_link"
    assert values(frame.findtext("pose")) == pytest.approx([0., 0., 0., 0., 0., DATA["lidar"]["nominal_yaw_rad"]])
    sensor = SDF.find("link[@name='lidar_link']/sensor")
    assert sensor.findtext("gz_frame_id") == "laser"
    assert values(sensor.findtext("pose")) == values(frame.findtext("pose"))
    assert ALIGNMENT["scan_height_status"] == DATA["lidar"]["scan_height_status"]
    assert ALIGNMENT["selected_navigation_projection"] == "chassis"


def test_reference_frame_change_preserves_all_visual_assembly_positions():
    anchor = DATA["cad_ros_base_origin_xyz_m"]
    positions = urdf_positions()
    for name, reference in ALIGNMENT["reference"]["links"].items():
        link = SDF.find(f"link[@name='{name}']")
        current = values(link.findtext("pose"))
        assert positions[name] == pytest.approx(current[:3], abs=2e-9)
        peer = URDF.find(f"link[@name='{name}']")
        assert len(link.findall("visual")) == len(reference["visuals"]) == len(peer.findall("visual"))
        for visual, original, urdf_visual in zip(link.findall("visual"), reference["visuals"], peer.findall("visual")):
            expected = [reference["pose"][i] + original[i] - anchor[i] for i in range(3)]
            local = values(visual.findtext("pose"))
            assert [current[i] + local[i] for i in range(3)] == pytest.approx(expected, abs=2e-9)
            origin = urdf_visual.find("origin")
            local_urdf = values(origin.get("xyz") if origin is not None else None, 3)
            assert [positions[name][i] + local_urdf[i] for i in range(3)] == pytest.approx(expected, abs=2e-9)


def test_reviewed_meshes_and_glb_geometry_remain_unchanged():
    paths = sorted((MODEL / "meshes").glob("*.dae"))
    assert len(paths) == 45
    assert {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in paths} == ALIGNMENT["reference"]["mesh_sha256"]
    manifest = json.loads((MODEL / "asset_manifest.json").read_text())
    for path, expected in manifest["sha256"].items():
        assert hashlib.sha256((PACKAGE / path).read_bytes()).hexdigest() == expected
    glb = (MODEL / "preview.glb").read_bytes()
    size = struct.unpack_from("<I", glb, 12)[0]
    assert hashlib.sha256(glb[20 + size:]).hexdigest() == ALIGNMENT["reference"]["glb_binary_sha256"]
    document = json.loads(glb[20:20 + size])
    anchor = DATA["cad_ros_base_origin_xyz_m"]
    for scene in document["scenes"]:
        assert len(scene["nodes"]) == 1
        root = document["nodes"][scene["nodes"][0]]
        assert root["name"] == "hardware_base_frame"
        assert root["translation"] == pytest.approx([-anchor[0], -anchor[2], anchor[1]])
