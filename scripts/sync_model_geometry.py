#!/usr/bin/env python3
"""Align the staged simulation asset to the inherited main CAD measurements.

This offline operation changes frames and collision dimensions only. It preserves
all 45 reviewed display meshes and does not configure ROS/Gazebo integration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import xml.etree.ElementTree as ET


def numbers(text, count=6):
    return list(map(float, (text or " ".join(["0"] * count)).split()))


def vector(values):
    return " ".join(f"{value:.12g}" for value in values)


def set_pose(element, pose):
    child = element.find("pose")
    if child is None:
        child = ET.SubElement(element, "pose")
    child.text = vector(pose)


def set_origin(element, pose):
    child = element.find("origin")
    if child is None:
        child = ET.SubElement(element, "origin")
    child.set("xyz", vector(pose[:3]))
    child.set("rpy", vector(pose[3:]))


def xml_write(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rebase_glb(path, anchor):
    data = path.read_bytes()
    assert data[:4] == b"glTF" and struct.unpack_from("<I", data, 4)[0] == 2
    size, kind = struct.unpack_from("<II", data, 12)
    assert kind == 0x4E4F534A
    doc = json.loads(data[20:20 + size])
    translation = [-anchor[0], -anchor[2], anchor[1]]  # ROS XYZ -> glTF X,Z,-Y.
    for scene in doc["scenes"]:
        roots = scene["nodes"]
        if len(roots) == 1 and doc["nodes"][roots[0]].get("name") == "hardware_base_frame":
            doc["nodes"][roots[0]]["translation"] = translation
        else:
            scene["nodes"] = [len(doc["nodes"])]
            doc["nodes"].append({"name": "hardware_base_frame", "translation": translation,
                                 "children": roots})
    chunk = json.dumps(doc, separators=(",", ":")).encode()
    chunk += b" " * (-len(chunk) % 4)
    tail = data[20 + size:]
    path.write_bytes(struct.pack("<4sII", b"glTF", 2, 20 + len(chunk) + len(tail))
                     + struct.pack("<II", len(chunk), kind) + chunk + tail)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    repo = args.repo.resolve()
    package = repo / "src/duojin01_description"
    model = package / "models/dual_arm_mobile"
    measurements = repo / "docs/chassis/measurements.json"
    data = json.loads(measurements.read_text())
    assert data["selected_navigation_projection"] == "chassis"
    anchor = data["cad_ros_base_origin_xyz_m"]
    sdf_path, urdf_path = model / "model.sdf", package / "urdf/dual_arm_mobile.urdf"
    sdf_root, urdf = ET.parse(sdf_path).getroot(), ET.parse(urdf_path).getroot()
    sdf = sdf_root.find("model")
    alignment = model / "hardware_alignment.json"
    if alignment.exists():
        reference = json.loads(alignment.read_text())["reference"]
    else:
        reference = {"asset_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True).strip(),
            "links": {}, "mesh_sha256": {p.name: sha(p) for p in sorted((model / "meshes").glob("*.dae"))}}
        for link in sdf.findall("link"):
            pose = numbers(link.findtext("pose"))
            assert pose[3:] == [0., 0., 0.] and not link.find("pose").attrib
            reference["links"][link.get("name")] = {
                "pose": pose, "visuals": [numbers(v.findtext("pose")) for v in link.findall("visual")],
                "inertial": numbers(link.findtext("inertial/pose")),
                "collisions": [numbers(c.findtext("pose")) for c in link.findall("collision")]}
    assert reference["mesh_sha256"] == {p.name: sha(p) for p in sorted((model / "meshes").glob("*.dae"))}
    if "glb_binary_sha256" not in reference:
        original_glb = subprocess.check_output(["git", "show", reference["asset_commit"]
            + ":src/duojin01_description/models/dual_arm_mobile/preview.glb"], cwd=repo)
        json_size = struct.unpack_from("<I", original_glb, 12)[0]
        reference["glb_binary_sha256"] = hashlib.sha256(original_glb[20 + json_size:]).hexdigest()
    new_poses = {}
    for link in sdf.findall("link"):
        name = link.get("name")
        original = reference["links"][name]
        pose = [original["pose"][i] - anchor[i] for i in range(3)] + [0., 0., 0.]
        wheel = data["wheels"].get(name.removesuffix("_wheel"))
        if name == "base_link":
            pose = [0.] * 6
        elif wheel:
            pose = wheel["center_xyz_m"] + [0., 0., 0.]
        elif name == "lidar_link":
            pose = data["lidar"]["nominal_scan_xyz_m"] + [0., 0., 0.]
        set_pose(link, pose)
        new_poses[name] = pose
        # These frames moved relative to their geometry; retain each visual's
        # original CAD zero pose and estimated centre of mass in the new frame.
        if name == "base_link" or wheel or name == "lidar_link":
            offset = [original["pose"][i] - anchor[i] - pose[i] for i in range(3)]
            peer = urdf.find(f"link[@name='{name}']")
            for target, source, u_target in zip(link.findall("visual"), original["visuals"], peer.findall("visual")):
                local = [source[i] + offset[i] for i in range(3)] + source[3:]
                set_pose(target, local)
                set_origin(u_target, local)
            local = [original["inertial"][i] + offset[i] for i in range(3)] + original["inertial"][3:]
            set_pose(link.find("inertial"), local)
            set_origin(peer.find("inertial"), local)
            for target, source, u_target in zip(link.findall("collision"), original["collisions"], peer.findall("collision")):
                if wheel:
                    target.find("geometry/cylinder/radius").text = vector([wheel["radial_envelope_radius_m"]])
                    target.find("geometry/cylinder/length").text = vector([wheel["width_m"]])
                    u_target.find("geometry/cylinder").set("radius", vector([wheel["radial_envelope_radius_m"]]))
                    u_target.find("geometry/cylinder").set("length", vector([wheel["width_m"]]))
                    local = [0., 0., 0.] + source[3:]
                elif name == "base_link":
                    bounds = data["chassis_body_collision_bounds_m"]
                    size = [bounds[1][i] - bounds[0][i] for i in range(3)]
                    target.find("geometry/box/size").text = vector(size)
                    u_target.find("geometry/box").set("size", vector(size))
                    local = [(bounds[1][i] + bounds[0][i]) / 2 for i in range(3)] + [0., 0., 0.]
                else:
                    local = [source[i] + offset[i] for i in range(3)] + source[3:]
                set_pose(target, local)
                set_origin(u_target, local)
    for joint in urdf.findall("joint"):
        if joint.find("parent").get("link") == "base_link":
            set_origin(joint, new_poses[joint.find("child").get("link")])
    # Preserve the staging model's link names and expose the hardware scan frame.
    yaw = data["lidar"]["nominal_yaw_rad"]
    laser = sdf.find("frame[@name='laser']")
    if laser is None:
        laser = ET.SubElement(sdf, "frame", name="laser", attached_to="lidar_link")
    set_pose(laser, [0., 0., 0., 0., 0., yaw])
    laser.find("pose").set("relative_to", "lidar_link")
    sensor = sdf.find("link[@name='lidar_link']/sensor[@name='lidar']")
    set_pose(sensor, [0., 0., 0., 0., 0., yaw])
    sensor.find("gz_frame_id").text = "laser"
    if urdf.find("link[@name='laser']") is None:
        ET.SubElement(urdf, "link", name="laser")
        joint = ET.SubElement(urdf, "joint", name="laser_frame_joint", type="fixed")
        ET.SubElement(joint, "parent", link="lidar_link")
        ET.SubElement(joint, "child", link="laser")
    set_origin(urdf.find("joint[@name='laser_frame_joint']"), [0., 0., 0., 0., 0., yaw])
    drive = sdf.find("plugin[@name='gz::sim::systems::MecanumDrive']")
    drive.find("wheelbase").text = vector([data["wheelbase_m"]])
    drive.find("wheel_separation").text = vector([data["track_width_m"]])
    drive.find("wheel_radius").text = vector([sum(w["radial_envelope_radius_m"] for w in data["wheels"].values()) / 4])
    xml_write(sdf_path, sdf_root)
    xml_write(urdf_path, urdf)
    rebase_glb(model / "preview.glb", anchor)
    glb = (model / "preview.glb").read_bytes()
    assert hashlib.sha256(glb[20 + struct.unpack_from("<I", glb, 12)[0]:]).hexdigest() == reference["glb_binary_sha256"]
    state = {"schema_version": 1, "source_main_commit": subprocess.check_output(
        ["git", "rev-parse", "main"], cwd=repo, text=True).strip(),
        "measurements_sha256": sha(measurements), "cad_ros_base_origin_xyz_m": anchor,
        "selected_navigation_projection": "chassis", "reference": reference,
        "visual_geometry_status": "All 45 imported DAE meshes unchanged; CAD zero-pose geometry preserved after base-frame translation",
        "scan_height_status": data["lidar"]["scan_height_status"],
        "rolling_radius_status": "Maximum roller envelope used as nominal simulator radius; effective physical rolling radius uncalibrated",
        "inertia_status": "Existing simulation estimates retained; physical mass/inertia not calibrated"}
    alignment.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n")
    manifest_path = model / "asset_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["hardware_alignment"] = {"main_commit": state["source_main_commit"], "record": "hardware_alignment.json"}
    manifest["sha256"]["models/dual_arm_mobile/hardware_alignment.json"] = sha(alignment)
    for relative in manifest["sha256"]:
        manifest["sha256"][relative] = sha(package / relative)
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    print("Aligned SDF, URDF and GLB to main geometry; 45 display mesh hashes preserved.")


if __name__ == "__main__":
    main()
