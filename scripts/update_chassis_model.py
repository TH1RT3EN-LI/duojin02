#!/usr/bin/env python3
"""Measure the reviewed Pack-and-Go tessellation and export hardware geometry.

Offline requirements: numpy, scipy, trimesh and Pillow. No simulator is needed.
Input is the validated extraction directory, not an approximate simulator rig.
"""

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial import ConvexHull
import trimesh


CAD_TO_ROS = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])
COLORS = {"shell": (.22, .24, .27), "metal": (.55, .59, .63),
          "battery": (.06, .33, .66), "wheel": (.07, .08, .09),
          "sensor": (.10, .12, .15)}
FASTENERS = re.compile("螺栓|螺钉|内六角|蘑菇头|螺柱|PCB|线端子")


def xml_write(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def vec(values):
    return " ".join(f"{float(v):.9g}" for v in values)


def export_dae(path, mesh, color):
    def add(parent, tag, text=None, **attrs):
        element = ET.SubElement(parent, tag, attrs)
        if text is not None:
            element.text = str(text)
        return element
    root = ET.Element("COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema",
                      version="1.4.1")
    asset = add(root, "asset")
    add(asset, "created", "2026-10-06T00:00:00Z")
    add(asset, "modified", "2026-10-06T00:00:00Z")
    add(asset, "unit", name="meter", meter="1")
    add(asset, "up_axis", "Z_UP")
    effect = add(add(root, "library_effects"), "effect", id="surface-effect")
    shader = add(add(add(effect, "profile_COMMON"), "technique", sid="common"), "phong")
    add(add(shader, "diffuse"), "color", vec([*color, 1]))
    add(add(shader, "specular"), "color", "0.15 0.15 0.15 1")
    add(add(shader, "shininess"), "float", "24")
    material = add(add(root, "library_materials"), "material", id="surface-material")
    add(material, "instance_effect", url="#surface-effect")
    geometry = add(add(root, "library_geometries"), "geometry", id="geometry")
    contents = add(geometry, "mesh")
    normals = mesh.vertex_normals[mesh.faces].copy()
    faces = np.repeat(mesh.face_normals[:, None, :], 3, axis=1)
    sharp = np.einsum("ijk,ijk->ij", normals, faces) < math.cos(math.radians(40))
    normals[sharp] = faces[sharp]
    normals = normals.reshape(-1, 3)
    for name, values in [("positions", mesh.vertices), ("normals", normals)]:
        source = add(contents, "source", id=name)
        add(source, "float_array", vec(values.reshape(-1)),
            id=name + "-array", count=str(values.size))
        accessor = add(add(source, "technique_common"), "accessor",
                       source="#" + name + "-array", count=str(len(values)), stride="3")
        for axis in "XYZ":
            add(accessor, "param", name=axis, type="float")
    vertices = add(contents, "vertices", id="vertices")
    add(vertices, "input", semantic="POSITION", source="#positions")
    triangles = add(contents, "triangles", count=str(len(mesh.faces)), material="surface")
    add(triangles, "input", semantic="VERTEX", source="#vertices", offset="0")
    add(triangles, "input", semantic="NORMAL", source="#normals", offset="1")
    indices = np.column_stack([mesh.faces.reshape(-1), np.arange(len(normals))]).reshape(-1)
    add(triangles, "p", " ".join(str(int(i)) for i in indices))
    scene = add(add(root, "library_visual_scenes"), "visual_scene", id="Scene")
    instance = add(add(scene, "node", id="node"), "instance_geometry", url="#geometry")
    add(add(add(instance, "bind_material"), "technique_common"), "instance_material",
        symbol="surface", target="#surface-material")
    add(add(root, "scene"), "instance_visual_scene", url="#Scene")
    xml_write(path, root)


def fit_wheel(mesh):
    """Use the concentric 9/11 mm hub rings, not the roller bounding box."""
    vertices = mesh.vertices
    midpoint = mesh.bounds[:, 0].mean()
    levels, counts = np.unique(np.round(vertices[:, 0], 6), return_counts=True)
    candidates = [(abs(level - midpoint), -int(count), level)
                  for level, count in zip(levels, counts)
                  if count >= 40 and abs(level - midpoint) < .0001]
    if not candidates:
        raise ValueError("No central hub-ring plane in the reviewed wheel geometry")
    level = min(candidates)[2]
    ring = vertices[np.abs(vertices[:, 0] - level) < .0000006]
    radial = ring[:, 1:3]
    mean = radial.mean(0)
    centered = radial - mean
    fit = np.linalg.lstsq(np.column_stack([2 * centered[:, 0], 2 * centered[:, 1],
                                          np.ones(len(centered))]),
                          (centered * centered).sum(1), rcond=None)[0]
    center = np.r_[ring[:, 0].mean(), mean + fit[:2]]
    radii = np.linalg.norm(radial - center[1:], axis=1)
    residual = np.minimum(abs(radii - .009), abs(radii - .011)).max()
    if residual > .0000001:
        raise ValueError(f"Hub-ring fit failed its native-geometry invariant: {residual}")
    return center, float(residual), len(ring)


def projection(vertices):
    hull = ConvexHull(vertices[:, :2])
    outline = vertices[hull.vertices, :2]
    angles = np.arange(8) * math.pi / 4
    normals = np.column_stack([np.cos(angles), np.sin(angles)])
    supports = np.ceil((outline @ normals.T).max(0) * 1000) / 1000
    polygon = np.array([np.linalg.solve(normals[[i, (i + 1) % 8]],
                                       supports[[i, (i + 1) % 8]]) for i in range(8)])
    assert np.all(outline @ normals.T <= supports + 1e-10)
    return {"bounds_m": [vertices.min(0).tolist(), vertices.max(0).tolist()],
            "convex_hull_xy_m": outline.tolist(), "area_m2": float(hull.volume),
            "support_normals_xy": normals.tolist(), "support_distances_m": supports.tolist(),
            "navigation_polygon_xy_m": np.round(polygon, 6).tolist()}


def draw_projection(path, data):
    image = Image.new("RGB", (1500, 1200), "#f8fafc")
    draw = ImageDraw.Draw(image)
    font_file = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font = ImageFont.truetype(font_file, 22)
    small = ImageFont.truetype(font_file, 18)
    title = ImageFont.truetype(font_file, 32)
    scale = 1500
    def xy(point):
        return (int(670 + point[0] * scale), int(600 - point[1] * scale))
    draw.text((55, 30), "最新 CAD：底盘、轮轴与地面投影", font=title, fill="#0f172a")
    draw.text((55, 82), "单位：米；+X 车头，+Y 左侧；原点为四轮轴中心的地面投影", font=font, fill="#475569")
    for value in np.arange(-.4, .501, .05):
        value = float(value)
        draw.line([xy((value, -.35)), xy((value, .35))], fill="#e2e8f0", width=1)
        draw.line([xy((-.35, value)), xy((.46, value))], fill="#e2e8f0", width=1)
    full = data["projections"]["whole_robot_saved_pose"]
    chassis = data["projections"]["chassis"]
    draw.polygon([xy(p) for p in full["navigation_polygon_xy_m"]], fill="#dbeafe", outline="#2563eb", width=4)
    draw.polygon([xy(p) for p in chassis["convex_hull_xy_m"]], fill="#e2e8f0", outline="#334155", width=3)
    outline = [xy(p) for p in full["convex_hull_xy_m"]]
    draw.line(outline + outline[:1], fill="#2563eb", width=2)
    labels = {"front_left": "左前", "rear_left": "左后", "front_right": "右前", "rear_right": "右后"}
    for name, wheel in data["wheels"].items():
        p = wheel["center_xyz_m"]
        radius, width = wheel["radial_envelope_radius_m"], wheel["width_m"]
        draw.rectangle([xy((p[0] - radius, p[1] + width / 2)),
                        xy((p[0] + radius, p[1] - width / 2))], fill="#475569", outline="#0f172a", width=2)
        x, y = xy(p)
        draw.line([(x - 8, y), (x + 8, y)], fill="#f59e0b", width=3)
        draw.line([(x, y - 8), (x, y + 8)], fill="#f59e0b", width=3)
        label = f"{labels[name]} ({p[0]:.4f}, {p[1]:.4f})"
        draw.text((x - 120, y - 68 if p[1] > 0 else y + 44), label, font=small, fill="#0f172a")
    p = data["lidar"]["axis_datum_xyz_m"]
    x, y = xy(p)
    draw.ellipse((x - 39, y - 39, x + 39, y + 39), outline="#dc2626", width=4)
    draw.text((x + 56, y - 20), f"N10P 转轴\nX={p[0]:.4f}, Y={p[1]:.4f}", font=small, fill="#b91c1c")
    x, y = xy((0, 0))
    draw.line([(x, y), (x + 115, y)], fill="#16a34a", width=4)
    draw.polygon([(x + 115, y), (x + 100, y - 7), (x + 100, y + 7)], fill="#16a34a")
    draw.line([(x, y), (x, y - 95)], fill="#16a34a", width=4)
    draw.text((x + 96, y + 8), "+X", font=small, fill="#166534")
    draw.text((x + 8, y - 105), "+Y", font=small, fill="#166534")
    chassis_size = np.diff(np.array(chassis["bounds_m"]), axis=0)[0]
    full_size = np.diff(np.array(full["bounds_m"]), axis=0)[0]
    lines = [f"底盘投影外包：{chassis_size[0]*1000:.1f} × {chassis_size[1]*1000:.1f} mm",
             f"整机保存姿态：{full_size[0]*1000:.1f} × {full_size[1]*1000:.1f} mm",
             f"轴距：{data['wheelbase_m']*1000:.1f} mm；轮距：{data['track_width_m']*1000:.1f} mm；轮宽：50.0 mm",
             "蓝色：整机投影与保守导航八边形；灰色：底盘；橙色：真实轮毂拟合轴心",
             "投影不含 Nav2 padding；整机足迹只对应 CAD 保存姿态，机械臂展开后需另行计算。"]
    for i, line in enumerate(lines):
        draw.text((55, 960 + i * 42), line, font=font, fill="#334155")
    image.save(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cad-analysis", type=Path, required=True)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--footprint", choices=["whole_robot_saved_pose", "chassis"], default="whole_robot_saved_pose")
    parser.add_argument("--lidar-zero-heading", choices=["unconfirmed", "forward", "backward", "left", "right"], default="unconfirmed")
    parser.add_argument("--lidar-scan-height", type=float, help="Measured optical centre height above the floor, in metres")
    args = parser.parse_args()
    source = args.cad_analysis.resolve()
    repo = args.repo.resolve()
    occurrences = json.loads((source.parent / "assembly_summary.json").read_text())["occurrences"]
    occurrences = [o for o in occurrences if not o["suppressed"] and not o["hidden"]]
    transforms = {k: np.array(v) for k, v in json.loads((source / "global_transforms.json").read_text()).items()}
    catalogue = json.loads((source / "mesh_catalogue.json").read_text())
    native = {mid: trimesh.load(source / "native_meshes" / (mid + ".ply"), process=False) for mid in catalogue}
    base_occ = next(o for o in occurrences if o["model_id"] == "94")
    matrix = transforms[base_occ["path"]]
    rotation = CAD_TO_ROS @ matrix[:3, :3]
    translation = CAD_TO_ROS @ matrix[:3, 3]
    components = native["94"].split(only_watertight=False)
    wheels = [p for p in components if len(p.faces) > 30000 and .095 < p.extents[1] < .11]
    assert len(wheels) == 4
    ground = -min((p.vertices @ rotation.T + translation)[:, 2].min() for p in wheels)
    ground_vector = np.array([0., 0., ground])
    wheel_records, wheel_meshes = {}, {}
    for mesh in wheels:
        center, residual, count = fit_wheel(mesh)
        world_center = rotation @ center + translation + ground_vector
        name = ("front" if world_center[0] > 0 else "rear") + "_" + ("left" if world_center[1] > 0 else "right")
        world_mesh = mesh.copy()
        world_mesh.apply_transform(np.block([[rotation, (translation + ground_vector)[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]))
        wheel_meshes[name] = world_mesh
        wheel_records[name] = {"cad_ros_center_xyz_m": world_center.tolist(), "width_m": float(mesh.extents[0]),
                               "radial_envelope_radius_m": float(np.linalg.norm(mesh.vertices[:, 1:3] - center[1:], axis=1).max()),
                               "hub_ring_fit_residual_m": residual, "hub_ring_vertices": count,
                               "cad_floor_clearance_m": float(world_mesh.bounds[0, 2])}
    anchor = np.mean([w["cad_ros_center_xyz_m"] for w in wheel_records.values()], axis=0)
    anchor[2] = 0.
    def world_mesh(occurrence):
        mesh = native[occurrence["model_id"]].copy()
        mesh.apply_transform(transforms[occurrence["path"]])
        mesh.apply_transform(np.block([[CAD_TO_ROS, np.zeros((3, 1))], [np.zeros((1, 3)), np.ones((1, 1))]]))
        mesh.apply_translation(ground_vector - anchor)
        return mesh
    all_vertices, chassis_vertices, groups, retained, removed = [], [], defaultdict(list), [], []
    for occurrence in occurrences:
        if occurrence["document_type"] != "PART":
            continue
        mesh = world_mesh(occurrence)
        all_vertices.append(mesh.vertices)
        if not occurrence["path"].startswith("1000"):
            continue
        chassis_vertices.append(mesh.vertices)
        mid, name = occurrence["model_id"], occurrence["display_name"]
        if mid in {"94", "114"}:
            continue
        if FASTENERS.search(name):
            removed.append({"name": name, "source_path": occurrence["path"]})
            continue
        kind = "shell" if "外壳" in name else "battery" if "电池" in name and "架" not in name else "metal"
        groups[kind].append(mesh)
        retained.append({"name": name, "model_id": mid, "source_path": occurrence["path"]})
    for component in components:
        if any(component is wheel for wheel in wheels):
            continue
        if max(component.extents) > .035 and sorted(component.extents)[-2] > .008:
            mesh = component.copy()
            mesh.apply_transform(np.block([[rotation, (translation + ground_vector - anchor)[:, None]], [np.zeros((1, 3)), np.ones((1, 1))]]))
            groups["shell"].append(mesh)
    lidar_occ = next(o for o in occurrences if o["model_id"] == "114")
    lidar_axis = CAD_TO_ROS @ transforms[lidar_occ["path"]][:3, 3] + ground_vector - anchor
    # Optical scan height is not a labelled datum in this extracted CAD.
    # Preserve a clearly documented nominal body-midplane until hardware calibration.
    scan_offset = float(native["114"].bounds[:, 0].mean())
    if args.lidar_scan_height is not None:
        scan_offset = args.lidar_scan_height - float(lidar_axis[2])
    lidar_scan = lidar_axis + np.array([0., 0., scan_offset])
    lidar_yaw = {"unconfirmed": 0., "forward": 0., "backward": math.pi,
                 "left": math.pi / 2, "right": -math.pi / 2}[args.lidar_zero_heading]
    meshes = repo / "src/duojin01_description/meshes"
    docs = repo / "docs/chassis"
    meshes.mkdir(parents=True, exist_ok=True)
    docs.mkdir(parents=True, exist_ok=True)
    assets = {}
    for kind, members in groups.items():
        mesh = trimesh.util.concatenate(members)
        name = "chassis_" + kind
        export_dae(meshes / (name + ".dae"), mesh, COLORS[kind])
        assets[name] = {"triangles": len(mesh.faces), "bounds_m": mesh.bounds.tolist()}
    for name, mesh in wheel_meshes.items():
        center = np.array(wheel_records[name]["cad_ros_center_xyz_m"]) - anchor
        mesh.apply_translation(-anchor - center)
        wheel_records[name]["center_xyz_m"] = center.tolist()
        export_dae(meshes / ("chassis_" + name + "_wheel.dae"), mesh, COLORS["wheel"])
        assets["chassis_" + name + "_wheel"] = {"triangles": len(mesh.faces), "bounds_m": mesh.bounds.tolist()}
    mesh = world_mesh(lidar_occ)
    mesh.apply_translation(-lidar_axis)
    export_dae(meshes / "n10plus.dae", mesh, COLORS["sensor"])
    assets["n10plus"] = {"triangles": len(mesh.faces), "bounds_m": mesh.bounds.tolist()}
    centers = {n: np.array(w["center_xyz_m"]) for n, w in wheel_records.items()}
    wheelbase = np.mean([centers["front_" + side][0] - centers["rear_" + side][0] for side in ["left", "right"]])
    track = np.mean([centers[end + "_left"][1] - centers[end + "_right"][1] for end in ["front", "rear"]])
    body_vertices = np.vstack([m.vertices for members in groups.values() for m in members])
    body_bounds = np.array([np.floor(body_vertices.min(0) * 1000) / 1000,
                            np.ceil(body_vertices.max(0) * 1000) / 1000])
    data = {"schema_version": 1, "source_archive": "PackAndGo 0000 双臂移动机器人-2026-09-21.zip",
            "source_archive_sha256": "d93e6bb2a731cceba861199dc13b86d40e639ec015137fd93c8b11a8519c5d6b",
            "native_inputs_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((source / "native_meshes").glob("*.ply"))},
            "assembly_inputs_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in [source.parent / "assembly_summary.json", source / "global_transforms.json", source / "mesh_catalogue.json"]},
            "coordinate_system": "metres; +X forward, +Y left, +Z up; origin at mean of four fitted wheel axes projected onto CAD floor",
            "cad_to_ros_rotation": CAD_TO_ROS.tolist(), "cad_ground_translation_m": ground,
            "cad_ros_base_origin_xyz_m": anchor.tolist(), "wheels": wheel_records,
            "wheelbase_m": float(wheelbase), "track_width_m": float(track),
            "chassis_body_collision_bounds_m": body_bounds.tolist(),
            "lidar": {"axis_datum_xyz_m": lidar_axis.tolist(), "nominal_scan_xyz_m": lidar_scan.tolist(),
                      "nominal_scan_offset_z_m": scan_offset,
                      "scan_height_status": "provided measured optical height" if args.lidar_scan_height is not None else "uncalibrated CAD body-midplane proxy; not a verified optical datum",
                      "nominal_yaw_rad": lidar_yaw, "zero_heading": args.lidar_zero_heading,
                      "yaw_status": "installation direction provided" if args.lidar_zero_heading != "unconfirmed" else "inherited forward-facing assumption; verify physical triangle zero marker",
                      "body_bottom_z_m": float(lidar_axis[2] + native["114"].bounds[0, 0]),
                      "body_top_z_m": float(lidar_axis[2] + native["114"].bounds[1, 0])},
            "imu": {"xyz_m": [0., 0., 0.], "rpy_rad": [0., 0., 0.], "status": "inherited uncalibrated transform; CAD has no identified IMU datum"},
            "projections": {"chassis": projection(np.vstack(chassis_vertices)),
                            "whole_robot_saved_pose": projection(np.vstack(all_vertices))},
            "selected_navigation_projection": args.footprint, "assets": assets, "retained_chassis_parts": retained,
            "removed_chassis_details": removed, "quality": "cleaned native tessellation; no further decimation; reconstructed semantic colors"}
    for name in assets:
        assets[name]["sha256"] = hashlib.sha256((meshes / (name + ".dae")).read_bytes()).hexdigest()
    (docs / "measurements.json").write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    properties = {"PI": math.pi, "lidar_mount_x": lidar_axis[0], "lidar_mount_y": lidar_axis[1], "lidar_mount_z": lidar_axis[2],
                  "lidar_scan_offset_z": scan_offset, "lidar_scan_yaw": lidar_yaw}
    for axis, center, size in zip("xyz", body_bounds.mean(0), body_bounds[1] - body_bounds[0]):
        properties["chassis_body_center_" + axis] = center
        properties["chassis_body_size_" + axis] = size
    for name, wheel in wheel_records.items():
        for axis, value in zip("xyz", wheel["center_xyz_m"]):
            properties[name + "_" + axis] = value
        properties[name + "_width"] = wheel["width_m"]
        properties[name + "_radius"] = wheel["radial_envelope_radius_m"]
    root = ET.Element("robot", name="chassis_geometry", attrib={"xmlns:xacro": "http://ros.org/wiki/xacro"})
    root.append(ET.Comment("Generated CAD geometry. See docs/chassis/measurements.json. Zero-heading status is recorded there; optical scan height and IMU still require hardware calibration."))
    for name, value in properties.items():
        ET.SubElement(root, "xacro:property", name=name, value=f"{value:.12g}")
    xml_write(repo / "src/duojin01_description/urdf/chassis_geometry.xacro", root)
    polygon = data["projections"][args.footprint]["navigation_polygon_xy_m"]
    nav = repo / "src/duojin01_bringup/config/nav2.yaml"
    text = nav.read_text()
    text, count = re.subn(r'footprint: "[^"\n]+"', 'footprint: "' + json.dumps(polygon, separators=(", ", ": ")) + '"', text)
    assert count == 2, count
    text, count = re.subn(r"inflation_radius: 0\.10\b", "inflation_radius: 0.35", text)
    assert count in {0, 2}, count
    nav.write_text(text)
    draw_projection(docs / "projection.png", data)
    print(json.dumps({"wheelbase_m": wheelbase, "track_width_m": track, "mesh_triangles": sum(a["triangles"] for a in assets.values()),
                      "lidar_axis_m": lidar_axis.tolist(), "footprint": polygon}, indent=2))


if __name__ == "__main__":
    main()
