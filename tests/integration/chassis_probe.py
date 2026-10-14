"""Source configuration access for isolated integration probes."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/duojin01_bringup'))
from duojin01_bringup.chassis_profile import component_file, default_profile, load_profile, render
