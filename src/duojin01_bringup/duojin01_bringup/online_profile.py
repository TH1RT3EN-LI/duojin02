"""Use lidar observations locally and the evolving SLAM map globally."""
import copy
import hashlib
import os
from pathlib import Path
import tempfile

import yaml
from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


def online_parameters(parameters):
    result = copy.deepcopy(parameters)
    local = result['local_costmap']['local_costmap']['ros__parameters']
    # A SLAM surface and its current odom-frame observation need not coincide.
    # Avoid treating both as independent local obstacles. Remembered lidar
    # returns remain active, including returns inside the lidar's blind zone.
    local['plugins'] = [name for name in local['plugins']
                        if local[name]['plugin'] != 'nav2_costmap_2d::StaticLayer']
    result['global_costmap']['global_costmap']['ros__parameters']['track_unknown_space'] = False
    for planner in result['planner_server']['ros__parameters']['planner_plugins']:
        result['planner_server']['ros__parameters'][planner]['allow_unknown'] = True
    return result


def for_online_file(source_file):
    payload = yaml.safe_dump(online_parameters(yaml.safe_load(Path(source_file).read_text()))).encode()
    directory = Path(tempfile.gettempdir()) / f'duojin01-online-{os.getuid()}'
    directory.mkdir(mode=0o700, exist_ok=True)
    destination = directory / (hashlib.sha256(payload).hexdigest() + '.yaml')
    if not destination.exists() or destination.read_bytes() != payload:
        with tempfile.NamedTemporaryFile(dir=directory, delete=False) as output:
            output.write(payload)
            temporary = output.name
        os.replace(temporary, destination)
    return str(destination)


class OnlineParameters(Substitution):
    def __init__(self, source_file):
        super().__init__()
        self.source_file = normalize_to_list_of_substitutions(source_file)

    def perform(self, context):
        return for_online_file(perform_substitutions(context, self.source_file))

    def describe(self):
        return 'Online lidar-local navigation parameters'
