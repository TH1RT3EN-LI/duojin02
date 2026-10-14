import copy
import hashlib
import os
from pathlib import Path
import tempfile

import yaml
from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from duojin02_bringup.lattice import for_resolution


def online_parameters(parameters, resolution, lattice_file):
    result = copy.deepcopy(parameters)
    local = result["local_costmap"]["local_costmap"]["ros__parameters"]

    local["plugins"] = [
        name for name in local["plugins"] if local[name]["plugin"] != "nav2_costmap_2d::StaticLayer"
    ]
    global_map = result["global_costmap"]["global_costmap"]["ros__parameters"]
    global_map["resolution"] = float(resolution)

    global_map["track_unknown_space"] = False
    lattice_path = for_resolution(lattice_file, resolution)
    planners = result["planner_server"]["ros__parameters"]
    for name in planners["planner_plugins"]:
        planner = planners[name]
        planner["allow_unknown"] = True
        if "lattice_filepath" in planner:
            planner["lattice_filepath"] = lattice_path
    return result


def for_online_file(source_file, slam_params_file, lattice_file):
    parameters = yaml.safe_load(Path(source_file).read_text())
    slam = yaml.safe_load(Path(slam_params_file).read_text())
    resolution = slam["slam_toolbox"]["ros__parameters"]["resolution"]
    payload = yaml.safe_dump(online_parameters(parameters, resolution, lattice_file)).encode()
    directory = Path(tempfile.gettempdir()) / f"duojin02-online-{os.getuid()}"
    directory.mkdir(mode=0o700, exist_ok=True)
    destination = directory / (hashlib.sha256(payload).hexdigest() + ".yaml")
    if not destination.exists() or destination.read_bytes() != payload:
        with tempfile.NamedTemporaryFile(dir=directory, delete=False) as output:
            output.write(payload)
            temporary = output.name
        os.replace(temporary, destination)
    return str(destination)


class OnlineParameters(Substitution):
    def __init__(self, source_file, slam_params_file, lattice_file):
        super().__init__()
        self.source_file = normalize_to_list_of_substitutions(source_file)
        self.slam_params_file = normalize_to_list_of_substitutions(slam_params_file)
        self.lattice_file = normalize_to_list_of_substitutions(lattice_file)

    def perform(self, context):
        return for_online_file(
            *(
                perform_substitutions(context, value)
                for value in (
                    self.source_file,
                    self.slam_params_file,
                    self.lattice_file,
                )
            )
        )

    def describe(self):
        return "Online lidar-local navigation parameters"
