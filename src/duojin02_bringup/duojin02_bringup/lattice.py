import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile


def for_resolution(source_file, resolution):
    source = Path(source_file)
    resolution = float(resolution)
    if not math.isfinite(resolution) or not 0.005 <= resolution <= 0.10:
        raise ValueError("Navigation map resolution must be between 0.005 and 0.10 m")
    data = json.loads(source.read_text())
    original = data["lattice_metadata"]["grid_resolution"]
    if abs(resolution - original) < 1e-9:
        return str(source)
    if data["lattice_metadata"]["motion_model"] != "omni":
        raise ValueError("Geometric scaling is supported only for this Omni primitive family")
    scaled = copy.deepcopy(data)
    factor = resolution / original
    scaled["lattice_metadata"]["grid_resolution"] = resolution
    scaled["lattice_metadata"]["turning_radius"] *= factor
    for primitive in scaled["primitives"]:
        for field in (
            "trajectory_radius",
            "trajectory_length",
            "arc_length",
            "straight_length",
        ):
            primitive[field] *= factor
        for pose in primitive["poses"]:
            pose[0] *= factor
            pose[1] *= factor

    payload = json.dumps(scaled, separators=(",", ":")).encode()
    directory = Path(tempfile.gettempdir()) / f"duojin02-lattices-{os.getuid()}"
    directory.mkdir(mode=0o700, exist_ok=True)
    destination = directory / (hashlib.sha256(payload).hexdigest() + ".json")
    if not destination.exists() or destination.read_bytes() != payload:
        with tempfile.NamedTemporaryFile(dir=directory, delete=False) as output:
            output.write(payload)
            temporary = output.name
        os.replace(temporary, destination)
    return str(destination)
