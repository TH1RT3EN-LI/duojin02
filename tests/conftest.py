"""Source paths for tests; simulation always uses its canonical checkout."""

import os
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
CHASSIS_PROFILE = ROOT / "src/duojin01_bringup/config/chassis.yaml"
sys.path.insert(0, str(ROOT / "src/duojin01_bringup"))


def simulation_package():
    source = Path(os.environ.get("DUOJIN_SIM_SOURCE", ROOT.with_name("duojin02-sim"))).expanduser().resolve()
    package = source / "simulation/src/duojin02_sim"
    if not (package / "duojin02_sim/chassis_crop.py").is_file():
        raise pytest.UsageError(f"Canonical simulation source unavailable: {source}; set DUOJIN_SIM_SOURCE")
    return package
