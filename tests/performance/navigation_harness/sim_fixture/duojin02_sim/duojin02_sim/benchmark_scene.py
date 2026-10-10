"""Challenge fixtures and independent geometric reference maps for Nav2 tests."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from .scene import generate, box, put


def generate_benchmark(description, profile, output, scenario, noise=True):
    scene, urdf = generate(description, profile, output, noise=noise, observer=False)
    case = json.loads(Path(scenario).read_text())
    bounds = case.get('bounds', [-3.0, 3.0, -2.5, 2.5])
    xmin, xmax, ymin, ymax = map(float, bounds)
    if not xmin < xmax or not ymin < ymax:
        raise ValueError('Scenario bounds must have positive width and height')
    root = ET.parse(scene)
    world = root.getroot().find('world')
    for model in list(world.findall('model')):
        if model.get('name') in ('low_target', 'raised_target', 'occluder') or model.get('name').startswith('grid_'):
            world.remove(model)
        elif 'bounds' in case and model.get('name') in (
                'wall_front', 'wall_back', 'wall_left', 'wall_right', 'floor'):
            world.remove(model)
    walls = [[xmin-.05, (ymin+ymax)/2, .1, ymax-ymin+.2],
             [xmax+.05, (ymin+ymax)/2, .1, ymax-ymin+.2],
             [(xmin+xmax)/2, ymin-.05, xmax-xmin, .1],
             [(xmin+xmax)/2, ymax+.05, xmax-xmin, .1]]
    if 'bounds' in case:
        box(world, 'floor', [(xmin+xmax)/2, (ymin+ymax)/2, -.05],
            [xmax-xmin+.6, ymax-ymin+.6, .1], '0.35 0.39 0.44 1')
        for name, (x,y,sx,sy) in zip(
                ('wall_back', 'wall_front', 'wall_right', 'wall_left'), walls):
            box(world, name, [x,y,1], [sx,sy,2], '0.42 0.48 0.54 1')
    else:
        # Retain the exact original fixtures for all existing frozen cases.
        walls = [[-3.05,0,.1,6],[3.05,0,.1,6],[0,-2.55,6,.1],[0,2.55,6,.1]]
    robot = world.find("model[@name='dual_arm']")
    if robot is None:
        robot = next(m for m in world.findall('model') if m.find("link[@name='lidar_link']") is not None)
    start = case.get('start', [-1.7, 0, 0])
    put(robot, 'pose', f'{start[0]} {start[1]} 0 0 0 {start[2]}')
    # Camera streams are excluded from the navigation CPU benchmark workload.
    for link in robot.findall('link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('type') == 'camera': link.remove(sensor)
    obstacles = []
    for i, obstacle in enumerate(case['obstacles']):
        x,y,sx,sy = obstacle
        name = f'challenge_{i}'
        box(world, name, [x,y,0.15], [sx,sy,0.30], '0.8 0.35 0.1 1')
        obstacles.append(dict(name=name,x=x,y=y,sx=sx,sy=sy))
    if case.get('dynamic'):
        box(world,'dynamic_obstacle',[0,1.8,0.15],[.22,.22,.30],'0.9 0.15 0.15 1')
    # Asymmetry makes heading and place identifiable; all maps use identical anchors.
    anchors = case.get('anchors', [[-2.2,1.5,.30,.17],[1.8,-1.8,.17,.45]])
    for i, (x,y,sx,sy) in enumerate(anchors):
        name = ('anchor_a', 'anchor_b')[i] if i < 2 else f'anchor_{i}'
        box(world,name,[x,y,.15],[sx,sy,.30],'0.1 0.6 0.4 1')
        obstacles.append(dict(name=name,x=x,y=y,sx=sx,sy=sy))
    root.write(scene,encoding='unicode',xml_declaration=True)
    resolution=case.get('map_resolution',.01)
    nx,ny=math.ceil((xmax-xmin+.2)/resolution),math.ceil((ymax-ymin+.2)/resolution)
    origin=[xmin-.1,ymin-.1,0.0]
    x=origin[0]+(np.arange(nx)+.5)*resolution
    y=origin[1]+(np.arange(ny)+.5)*resolution
    grid=np.full((ny,nx),254,dtype=np.uint8)
    # Conservative cell intersection: a cell touching a physical obstacle is occupied.
    for cx,cy,sx,sy in walls+[[o[k] for k in ('x','y','sx','sy')] for o in obstacles]:
        mask=(np.abs(x[None,:]-cx)<=sx/2+resolution/2-1e-8)&(np.abs(y[:,None]-cy)<=sy/2+resolution/2-1e-8)
        grid[mask]=0
    output=Path(output)
    (output/'map.pgm').write_bytes(f'P5\n{nx} {ny}\n255\n'.encode()+grid[::-1].tobytes())
    (output/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=resolution,origin=origin,
                        negate=0,occupied_thresh=.65,free_thresh=.25)))
    case.update(obstacle_reference=obstacles,scene=scene,map=str(output/'map.yaml'),
                oracle_walls=walls,sensor_scope='2D lidar, chassis projection, kinematic base')
    (output/'case.json').write_text(json.dumps(case,indent=2))
    return scene,urdf,str(output/'map.yaml'),case
