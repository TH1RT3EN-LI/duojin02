"""Map/query geometry validation, using the shared map-export contract."""
from pathlib import Path
import yaml
from duojin02_slam_tools.scan_contract import (  # public compatibility imports
    SCHEMA, MEASUREMENT_CONTRACT, DEFAULTS, contract, from_parameter_file,
    legacy_compatible, validate_map_contract,
)


def validate_localization_launch(context):
    from launch.substitutions import LaunchConfiguration

    source = Path(LaunchConfiguration("params_file").perform(context))
    parameters = yaml.safe_load(source.read_text())["amcl"]["ros__parameters"]
    validate_map_contract(LaunchConfiguration("map").perform(context), parameters)
    return []
