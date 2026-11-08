"""Map/query time and beam geometry may only be selected as a matching pair."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "src/duojin02_bringup"))
sys.path.insert(0, str(Path(__file__).parents[1] / "src/duojin02_slam_tools"))
from duojin02_bringup.scan_geometry import DEFAULTS, contract, validate_map_contract


def test_legacy_map_keeps_identity_convention(tmp_path):
    validate_map_contract(tmp_path / "map.yaml", {})


@pytest.mark.parametrize("parameters", [
    {"scan_motion_compensation": True},
    {"scan_reference_fraction": 0.5},
    {"capture_time_offset_sec": -0.11,
     "capture_time_mapping_id": "sensitivity-v1", "capture_time_mapping_source": "uncalibrated"},
])
def test_changed_geometry_rejects_legacy_map(tmp_path, parameters):
    with pytest.raises(ValueError, match="lacks its scan geometry"):
        validate_map_contract(tmp_path / "map.yaml", parameters)


def test_nonzero_offset_requires_named_evidence():
    with pytest.raises(ValueError, match="requires explicit"):
        contract({"capture_time_offset_sec": -0.11})


def test_contract_does_not_claim_missing_physical_evidence():
    value = contract({"capture_time_mapping_id": "custom-labelled-calibration",
                      "capture_time_mapping_source": "unverified text"})
    assert value["capture_time_evidence_status"] == "uncalibrated"
    assert value["beam_order_evidence_status"] == "original-index-increasing-time-assumed"
    assert value["sensor_no_return_policy"] == "max-or-positive-infinity"


def test_v1_compensated_map_requires_rebuild(tmp_path):
    old = {"schema": "duojin02.scan_geometry.v1", **DEFAULTS,
           "scan_motion_compensation": True}
    (tmp_path / "map.scan_geometry.json").write_text(json.dumps(old))
    with pytest.raises(ValueError, match="contracts differ"):
        validate_map_contract(tmp_path / "map.yaml", old)


def test_v1_rigid_legacy_map_is_explicitly_compatible(tmp_path):
    (tmp_path / "map.scan_geometry.json").write_text(json.dumps(
        {"schema": "duojin02.scan_geometry.v1", **DEFAULTS}))
    validate_map_contract(tmp_path / "map.yaml", {})


def test_matching_map_and_query_accepts(tmp_path):
    parameters = {**DEFAULTS, "scan_motion_compensation": True}
    (tmp_path / "map.scan_geometry.json").write_text(json.dumps(contract(parameters)))
    validate_map_contract(tmp_path / "map.yaml", parameters)


@pytest.mark.parametrize("enabled", [False, True])
def test_motion_model_declares_conditional_uncalibrated_processing(tmp_path, enabled):
    parameters = {"scan_motion_compensation": enabled}
    value = contract(parameters)
    model = value["motion_model"]
    assert model["trajectory"] == ("se2_interpolated" if enabled else "rigid_at_reference")
    assert model["temporal_covariance"] == "not_supplied"
    assert model["uncertainty"] == "conditional_not_marginalized"
    assert model["evidence_independence"] == "unknown"
    assert model["shared_motion"] == "conditioned_not_marginalized"
    assert model["beam_noise_model"] == "consumer_defined"
    assert "beam_independence" not in model
    # Existing v2 maps are interpreted as legacy deterministic, never perfect.
    del value["motion_model"]
    (tmp_path / "map.scan_geometry.json").write_text(json.dumps(value))
    validate_map_contract(tmp_path / "map.yaml", parameters)


def test_unsupported_uncertainty_model_fails_closed(tmp_path):
    value = contract({})
    value["motion_model"]["uncertainty"] = "calibrated_marginal"
    (tmp_path / "map.scan_geometry.json").write_text(json.dumps(value))
    with pytest.raises(ValueError, match="contracts differ"):
        validate_map_contract(tmp_path / "map.yaml", {})
    with pytest.raises(ValueError, match="unsupported motion_model"):
        contract(value)


def test_deskew_map_rejects_legacy_query(tmp_path):
    (tmp_path / "map.scan_geometry.json").write_text(json.dumps(contract({"scan_motion_compensation": True})))
    with pytest.raises(ValueError, match="contracts differ"):
        validate_map_contract(tmp_path / "map.yaml", {})


def test_time_mapping_is_part_of_contract(tmp_path):
    parameters = {"capture_time_offset_sec": 0.01,
                  "capture_time_mapping_id": "device-test-v1",
                  "capture_time_mapping_source": "calibration.csv"}
    (tmp_path / "map.scan_geometry.json").write_text(json.dumps(contract(parameters)))
    with pytest.raises(ValueError, match="contracts differ"):
        validate_map_contract(tmp_path / "map.yaml", {**parameters, "capture_time_offset_sec": 0.02})
