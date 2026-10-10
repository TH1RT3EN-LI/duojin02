"""Create runtime extensions of the reviewed CAD assets; never modify source files."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def put(parent, path, value):
    node = parent
    for part in path.split('/'):
        found = node.find(part)
        node = found if found is not None else ET.SubElement(node, part)
    node.text = str(value)
    return node


def plugin(parent, filename, name, **parameters):
    node = ET.SubElement(parent, 'plugin', filename='gz-sim-' + filename + '-system',
                         name='gz::sim::systems::' + name)
    for key, value in parameters.items():
        put(node, key, value)
    return node


def box(world, name, xyz, size, color, collision=True):
    model = ET.SubElement(world, 'model', name=name)
    put(model, 'static', 'true')
    put(model, 'pose', ' '.join(map(str, [*xyz, 0, 0, 0])))
    link = ET.SubElement(model, 'link', name='body')
    if collision:
        put(ET.SubElement(link, 'collision', name='collision'), 'geometry/box/size', ' '.join(map(str, size)))
    visual = ET.SubElement(link, 'visual', name='visual')
    put(visual, 'geometry/box/size', ' '.join(map(str, size)))
    put(visual, 'material/diffuse', color)
    put(visual, 'material/ambient', color)
    put(visual, 'material/pbr/metal/metalness', 0.0)
    put(visual, 'material/pbr/metal/roughness', 0.75)
    return model


def generate(description, profile_path, output, noise=True, observer=True):
    description, output = Path(description), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    profile = yaml.safe_load(Path(profile_path).read_text())
    lidar = profile['lidar']
    model = ET.parse(description / 'models/dual_arm_mobile/model.sdf').getroot().find('model')
    # A single plugin owns base motion. Cylindrical wheel contacts do not model rollers.
    for node in list(model.findall('plugin')):
        if node.get('name').endswith(('MecanumDrive', 'VelocityControl')):
            model.remove(node)
    plugin(model, 'velocity-control', 'VelocityControl', topic='/sim/cmd_vel')
    plugin(model, 'duojin-odometry-publisher', 'OdometryPublisher', odom_frame='odom',
           robot_base_frame='base_footprint', odom_publish_frequency=50,
           dimensions=3, odom_topic='/sim/ground_truth/odom', tf_topic='/sim/ground_truth/tf')
    for name in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        plugin(model, 'joint-controller', 'JointController', joint_name=name + '_wheel_joint',
               topic='/sim/wheels/' + name, use_force_commands='false')
        # Low traction in kinematic mode avoids pretending cylinders are mecanum rollers.
        wheel = model.find(f"link[@name='{name}_wheel']")
        for collision in wheel.findall('collision'):
            put(collision, 'surface/friction/ode/mu', 0.01)
            put(collision, 'surface/friction/ode/mu2', 0.01)
    for visual in model.findall('.//visual'):
        material = visual.find('material')
        settings = profile['materials'][visual.get('name')]
        for key, value in settings.items():
            put(material, 'pbr/metal/' + key, value)
    sensor = model.find("link[@name='lidar_link']/sensor")
    lidar_z = float(model.findtext("link[@name='lidar_link']/pose").split()[2])
    scan_offset = lidar['scan_height_m'] - lidar_z
    put(sensor, 'pose', '0 0 ' + str(scan_offset) + ' 0 0 0')
    put(model.find("frame[@name='laser']"), 'pose', '0 0 ' + str(scan_offset) + ' 0 0 0')
    put(sensor, 'topic', '/sim/lidar/raw')
    put(sensor, 'always_on', 'true')
    put(sensor, 'update_rate', lidar['rate_hz'])
    put(sensor, 'lidar/scan/horizontal/samples', lidar['samples'])
    put(sensor, 'lidar/scan/horizontal/min_angle', -math.pi)
    # No duplicated -pi/+pi ray; hardware grid has 540 distinct bins.
    put(sensor, 'lidar/scan/horizontal/max_angle', -math.pi + 2*math.pi*(lidar['samples']-1)/lidar['samples'])
    put(sensor, 'lidar/range/min', lidar['optical_range_min_m'])
    put(sensor, 'lidar/visibility_mask', 1)
    # Opaque CAD housing has no optical window. Exclude only this housing from
    # its own beam; it remains visible to cameras. Chassis and pillars stay visible.
    for visual in model.findall("link[@name='lidar_link']/visual"):
        put(visual, 'visibility_flags', 2)
    put(sensor, 'lidar/range/max', lidar['optical_range_max_m'])
    put(sensor, 'lidar/range/resolution', 0.001)
    if noise:
        node = ET.SubElement(sensor.find('lidar'), 'noise', type='gaussian')
        put(node, 'mean', 0.0)
        put(node, 'stddev', lidar['noise_stddev_m'])
    for camera in model.findall(".//sensor[@type='camera']"):
        put(camera, 'update_rate', profile['camera']['rate_hz'])
        for key in ('width', 'height'):
            put(camera, 'camera/image/' + key, profile['camera'][key])
    imu = ET.SubElement(model.find("link[@name='base_link']"), 'sensor', name='imu', type='imu')
    put(imu, 'topic', '/sim/imu/raw')
    put(imu, 'gz_frame_id', 'imu_link')
    put(imu, 'update_rate', 100)
    put(imu, 'always_on', 'true')
    ET.SubElement(imu, 'imu')
    root = ET.Element('sdf', version='1.9')
    world = ET.SubElement(root, 'world', name='duojin02_lab')
    physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
    put(physics, 'max_step_size', 0.001)
    put(physics, 'real_time_factor', 1)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                            ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu')]:
        plugin(world, filename, name)
    plugin(world, 'sensors', 'Sensors', render_engine='ogre2')
    put(world, 'scene/ambient', '0.45 0.45 0.45 1')
    put(world, 'scene/background', '0.08 0.10 0.14 1')
    put(world, 'scene/shadows', 'true')
    light = ET.SubElement(world, 'light', name='key', type='directional')
    put(light, 'pose', '0 0 5 0 0 0')
    put(light, 'diffuse', '0.9 0.88 0.84 1')
    put(light, 'specular', '0.5 0.5 0.5 1')
    put(light, 'direction', '-0.4 0.5 -1')
    put(light, 'cast_shadows', 'true')
    box(world, 'floor', [0, 0, -0.05], [12, 10, 0.1], '0.35 0.39 0.44 1')
    # Four flat targets have analytically known nearest surfaces.
    box(world, 'wall_front', [3.05, 0, 1], [0.1, 6, 2], '0.68 0.73 0.78 1')
    box(world, 'wall_back', [-3.05, 0, 1], [0.1, 6, 2], '0.25 0.32 0.4 1')
    box(world, 'wall_left', [0, 2.55, 1], [6, 0.1, 2], '0.22 0.4 0.55 1')
    box(world, 'wall_right', [0, -2.55, 1], [6, 0.1, 2], '0.58 0.42 0.26 1')
    box(world, 'low_target', [1.25, -0.55, 0.035], [0.25, 0.25, 0.07], '0.9 0.55 0.1 1')
    box(world, 'raised_target', [1.25, 0.55, 0.48], [0.25, 0.25, 0.2], '0.1 0.65 0.55 1')
    box(world, 'occluder', [1.5, 1.4, 0.45], [0.3, 0.4, 0.9], '0.86 0.3 0.12 1')
    # Decorative floor markings are below the lidar plane, visuals only.
    for i in range(-5, 6):
        box(world, 'grid_x_' + str(i), [i*0.5, 0, 0.0005], [0.008, 5, 0.001], '0.48 0.52 0.56 1', False)
        box(world, 'grid_y_' + str(i), [0, i*0.5, 0.0006], [6, 0.008, 0.001], '0.48 0.52 0.56 1', False)
    world.append(model)
    if observer:
        view = ET.SubElement(world, 'model', name='observer')
        put(view, 'static', 'true')
        link = ET.SubElement(view, 'link', name='camera')
        camera = ET.SubElement(link, 'sensor', name='overview', type='camera')
        # Camera +X points toward the full robot at (0,0,0.55).
        put(camera, 'pose', '2.1 -2.6 1.9 0 0.39 2.25')
        put(camera, 'topic', '/sim/overview/image')
        put(camera, 'gz_frame_id', 'overview_optical_frame')
        put(camera, 'update_rate', profile['observer']['rate_hz'])
        put(camera, 'always_on', 'true')
        put(camera, 'camera/horizontal_fov', 0.80)
        for key in ('width', 'height'):
            put(camera, 'camera/image/' + key, profile['observer'][key])
        put(camera, 'camera/clip/near', 0.05)
        put(camera, 'camera/clip/far', 30)
    ET.indent(root)
    scene = output / 'lab.sdf'
    ET.ElementTree(root).write(scene, encoding='unicode', xml_declaration=True)
    # RSP gets a simulation-only IMU alias; hardware xacro remains unchanged.
    urdf = ET.parse(description / 'urdf/dual_arm_mobile.urdf')
    laser_joint = next(j for j in urdf.getroot().findall('joint') if j.find('child').get('link') == 'laser')
    laser_joint.find('origin').set('xyz', '0 0 ' + str(scan_offset))
    ET.SubElement(urdf.getroot(), 'link', name='imu_link')
    joint = ET.SubElement(urdf.getroot(), 'joint', name='sim_imu_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='imu_link')
    ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    return str(scene), ET.tostring(urdf.getroot(), encoding='unicode')
