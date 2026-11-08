"""Fail closed when map and query use different scan geometry/time conventions."""
import json
import math
from pathlib import Path

import yaml

SCHEMA = "duojin02.scan_geometry.v2"
MEASUREMENT_CONTRACT = {
    "measurement_geometry_version": "typed-rays.v2",
    "sensor_no_return_policy": "max-or-positive-infinity",
    "capture_time_evidence_status": "uncalibrated",
    "beam_order_evidence_status": "original-index-increasing-time-assumed",
}
DEFAULTS = {
    "scan_motion_compensation": False,
    "capture_time_offset_sec": 0.0,
    "capture_time_mapping_id": "header-first-ray-assumed-v1",
    "capture_time_mapping_source": "",
    "scan_reference_fraction": 0.0,
    "scan_max_tf_gap_sec": 0.1,
}


def motion_model(parameters):
    """Describe the implemented conditional model, without certifying TF noise."""
    return {
        "version": "conditional-trajectory.v1",
        "trajectory": ("se2_interpolated" if parameters.get("scan_motion_compensation", False)
                       else "rigid_at_reference"),
        "provider": "tf_pose_history",
        "temporal_covariance": "not_supplied",
        "uncertainty": "conditional_not_marginalized",
        "evidence_independence": "unknown",
        "shared_motion": "conditioned_not_marginalized",
        "beam_noise_model": "consumer_defined",
    }


def contract(parameters):
    result = {"schema": SCHEMA, **MEASUREMENT_CONTRACT,
              **{key: parameters.get(key, value) for key, value in DEFAULTS.items()}}
    if type(result["scan_motion_compensation"]) is not bool:
        raise ValueError("scan_motion_compensation must be a YAML boolean")
    result["motion_model"] = motion_model(result)
    if "motion_model" in parameters and parameters["motion_model"] != result["motion_model"]:
        raise ValueError("unsupported motion_model; only conditional deterministic trajectories are implemented")
    for key in ("capture_time_offset_sec", "scan_reference_fraction", "scan_max_tf_gap_sec"):
        value = result[key]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"{key} must be finite")
    if not 0 <= result["scan_reference_fraction"] <= 1 or result["scan_max_tf_gap_sec"] <= 0:
        raise ValueError("invalid scan reference fraction or TF gap")
    if not isinstance(result["capture_time_mapping_id"], str) or not result["capture_time_mapping_id"]:
        raise ValueError("capture_time_mapping_id must be nonempty")
    if not isinstance(result["capture_time_mapping_source"], str):
        raise ValueError("capture_time_mapping_source must be text")
    if result["capture_time_offset_sec"] != 0 and (
        result["capture_time_mapping_id"] == DEFAULTS["capture_time_mapping_id"]
        or not result["capture_time_mapping_source"].strip()
    ):
        raise ValueError("nonzero capture offset requires explicit mapping ID and evidence source")
    return result


def from_parameter_file(path, node):
    data = yaml.safe_load(Path(path).read_text())
    return contract(data[node]["ros__parameters"])


def legacy_compatible(value):
    return all(value.get(key) == DEFAULTS[key] for key in DEFAULTS if key != "scan_max_tf_gap_sec")


def validate_map_contract(map_yaml, parameters):
    """Old maps are usable only under the historical identity/no-deskew convention."""
    expected = contract(parameters)
    prefix = Path(map_yaml).expanduser().resolve().with_suffix("")
    bundle_sidecar = Path(map_yaml).parent / "scan_geometry.json"
    sidecar = (bundle_sidecar if (Path(map_yaml).parent / "snapshot.json").is_file()
               else prefix.with_name(prefix.name + ".scan_geometry.json"))
    if not sidecar.is_file():
        if legacy_compatible(expected):
            return
        raise ValueError(
            f"Map lacks its scan geometry contract: {sidecar}. Rebuild the map with the same "
            "time mapping and scan_motion_compensation setting before localization."
        )
    actual = json.loads(sidecar.read_text())
    # Historical v2 rays used this same deterministic algorithm. Missing model
    # metadata means legacy conditional processing, never zero motion error.
    if actual.get("schema") == SCHEMA and "motion_model" not in actual:
        actual["motion_model"] = motion_model(actual)
    # Only the explicitly historical rigid path can reuse a v1 sidecar. A v1
    # compensated map may contain mixed coarse/fine geometry and must be rebuilt.
    if actual.get("schema") == "duojin02.scan_geometry.v1":
        if not actual.get("scan_motion_compensation", True) and legacy_compatible(actual):
            actual = contract(actual)
    if actual != expected:
        raise ValueError(
            f"Map/query scan geometry contracts differ: {sidecar}. Select matching mapping "
            "and localization parameters or rebuild the map."
        )
