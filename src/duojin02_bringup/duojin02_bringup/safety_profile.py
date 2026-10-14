from pathlib import Path

import yaml


def safety_defaults():
    source = Path(__file__).resolve().parents[1] / "config/safety.yaml"
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory

        source = Path(get_package_share_directory("duojin02_bringup")) / "config/safety.yaml"
    parameters = yaml.safe_load(source.read_text())
    return {
        **parameters["/**"]["ros__parameters"],
        **parameters["velocity_guard"]["ros__parameters"],
    }


def mux_parameters(config_path):
    parameters = yaml.safe_load(Path(config_path).read_text())["twist_mux"]["ros__parameters"]
    timeout = safety_defaults()["command_timeout_s"]
    for source in parameters["topics"].values():
        source["timeout"] = timeout
    return parameters
