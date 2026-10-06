"""Render the actual exported hardware meshes with Blender, without a simulator."""
import bpy
import json
import math
from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[1]
DATA = json.loads((ROOT / "docs/chassis/measurements.json").read_text())
COLORS = {"shell": (.22, .24, .27), "metal": (.55, .59, .63),
          "battery": (.06, .33, .66), "wheel": (.07, .08, .09),
          "sensor": (.10, .12, .15)}
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
bpy.ops.wm.read_factory_settings(use_empty=True)
temporary = tempfile.TemporaryDirectory(prefix="duojin02-chassis-preview-")
for name in DATA["assets"]:
    document = ET.parse(ROOT / "src/duojin01_description/meshes" / (name + ".dae"))
    values = list(map(float, document.findtext('.//c:source[@id="positions"]/c:float_array', namespaces=NS).split()))
    vertices = [values[i:i + 3] for i in range(0, len(values), 3)]
    indices = list(map(int, document.findtext('.//c:triangles/c:p', namespaces=NS).split()))
    faces = [indices[i:i + 6:2] for i in range(0, len(indices), 6)]
    # Use Blender's validated PLY importer for the dense engineering meshes.
    ply = Path(temporary.name) / (name + ".ply")
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(vertices)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              f"element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header\n")
    with ply.open("wb") as output:
        output.write(header.encode("ascii"))
        output.write(b"".join(struct.pack("<3f", *v) for v in vertices))
        output.write(b"".join(struct.pack("<B3i", 3, *f) for f in faces))
    bpy.ops.wm.ply_import(filepath=str(ply))
    obj = bpy.context.object
    obj.name = name
    geometry = obj.data
    if name == "n10plus":
        obj.location = DATA["lidar"]["axis_datum_xyz_m"]
        color = COLORS["sensor"]
    elif name.endswith("_wheel"):
        obj.location = DATA["wheels"][name.removeprefix("chassis_").removesuffix("_wheel")]["center_xyz_m"]
        color = COLORS["wheel"]
    else:
        color = COLORS[name.removeprefix("chassis_")]
    material = bpy.data.materials.new(name)
    material.diffuse_color = (*color, 1)
    material.use_nodes = True
    material.node_tree.nodes.clear()
    node = material.node_tree.nodes.new("ShaderNodeBsdfPrincipled")
    output = material.node_tree.nodes.new("ShaderNodeOutputMaterial")
    material.node_tree.links.new(node.outputs["BSDF"], output.inputs["Surface"])
    node.inputs["Base Color"].default_value = (*color, 1)
    node.inputs["Roughness"].default_value = .5
    node.inputs["Metallic"].default_value = .45 if name.endswith("metal") else .08
    geometry.materials.clear()
    geometry.materials.append(material)
    for polygon in geometry.polygons:
        polygon.material_index = 0
        polygon.use_smooth = True

def aim(obj, point):
    obj.rotation_euler = (Vector(point) - obj.location).to_track_quat("-Z", "Y").to_euler()

bpy.ops.mesh.primitive_plane_add(size=200, location=(0, 0, -.001))
floor = bpy.data.materials.new("floor")
floor.diffuse_color = (.72, .75, .79, 1)
bpy.context.object.data.materials.append(floor)
for position, power in [((1, -1.2, 1.5), 65), ((-.9, -.5, 1), 40), ((0, 1.2, 1.3), 55)]:
    bpy.ops.object.light_add(type="AREA", location=position)
    light = bpy.context.object
    light.data.energy = power
    light.data.size = 1.0
    aim(light, (0, 0, .1))
bpy.ops.object.camera_add(location=(.75, -.9, .65))
camera = bpy.context.object
aim(camera, (0, 0, .1))
camera.data.type = "ORTHO"
camera.data.ortho_scale = .70
scene = bpy.context.scene
scene.camera = camera
scene.render.engine = "CYCLES"
scene.cycles.samples = 48
scene.cycles.use_denoising = True
scene.world = bpy.data.worlds.new("studio")
scene.world.color = (.35, .35, .35)
scene.render.resolution_x = 1200
scene.render.resolution_y = 900
scene.render.resolution_percentage = 100
scene.render.image_settings.file_format = "PNG"
scene.render.filepath = str(ROOT / "docs/chassis/model.png")
bpy.ops.render.render(write_still=True)
temporary.cleanup()
