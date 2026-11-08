#!/usr/bin/env python3
"""Exercise the installed GPU AMCL's new numerical and timing contracts.

This is a synthetic integration check, not LILocBench accuracy validation.
"""
import argparse
import json
from pathlib import Path
import re
from types import SimpleNamespace

import numpy as np
import yaml

from cuda_amcl_replay_probe import fixture, require, rolling_curve_fixture, run_arm
from ros_isolation import configure_isolation


def reject_configuration(args, parameters, output, reason):
    """Require rejection by the actual node's lifecycle configure transition."""
    import os
    import signal
    import subprocess
    import rclpy
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from rclpy.node import Node

    output.mkdir()
    parameters = {**parameters, "use_sim_time": True, "observation_backend": "cuda"}
    path = output / "input.yaml"
    path.write_text(yaml.safe_dump({"amcl": {"ros__parameters": parameters}}))
    logfile = output / "node.log"
    rclpy.init(args=[])
    node = Node("amcl_invalid_configuration_observer")
    client = node.create_client(ChangeState, "/amcl/change_state")
    response = None
    with logfile.open("w") as log:
        process = subprocess.Popen([str(args.executable), "--ros-args", "--params-file", str(path)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            require(client.wait_for_service(timeout_sec=15.), "invalid-case lifecycle service unavailable")
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
            require(future.done() and future.result() is not None, "invalid-case configure did not respond")
            response = future.result().success
            require(not response, "actual AMCL node accepted invalid configuration")
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()
    text = logfile.read_text()
    require(reason.lower() in text.lower(), f"configuration rejected without expected reason {reason!r}")
    return {"configure_success": response, "reason_verified": reason, "status": "passed"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--expected-prefix", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    domain = configure_isolation()
    base = yaml.safe_load(args.parameters.read_text())["amcl"]["ros__parameters"]
    base.update({"filter_diagnostics_log_every": 1})
    scenarios = [
        ("legacy", {}, False),
        ("ess_skip", {"resample_ess_ratio": 1e-12}, False),
        ("temperature_confidence", {"likelihood_temperature_beta": .5,
            "kld_confidence_probability": .95}, False),
        ("deskew_pending", {"scan_motion_compensation": True,
            "scan_reference_fraction": .5}, True),
        ("capture_offset_pending", {"capture_time_offset_sec": .02,
            "capture_time_mapping_id": "synthetic-positive-offset-v1",
            "capture_time_mapping_source": "analytic fixture; not a sensor calibration"}, False),
        ("deskew_curve_reference", {"scan_motion_compensation": True,
            "scan_reference_fraction": .5}, True),
        ("zero_offset_provenance_reference", {"capture_time_offset_sec": 0.,
            "scan_reference_fraction": 0., "scan_motion_compensation": False,
            "capture_time_mapping_id": "synthetic-zero-offset-calibrated-v1",
            "capture_time_mapping_source": "analytic zero-offset calibration fixture"}, False),
    ]
    report = {"scope": "installed GPU AMCL synthetic contract checks",
              "ros_domain_id": domain, "scenarios": {}, "status": "running"}
    try:
        for name, overrides, deskew in scenarios:
            output = args.output / name
            output.mkdir()
            parameters = {**base, **overrides}
            if "kld_confidence_probability" in overrides:
                parameters.pop("pf_z", None)
            path = output / "input.yaml"
            path.write_text(yaml.safe_dump({"amcl": {"ros__parameters": parameters}}))
            if name in ("deskew_curve_reference", "zero_offset_provenance_reference"):
                inputs = rolling_curve_fixture(12, rolling=name == "deskew_curve_reference")
            else:
                inputs = fixture(12)
            if name == "zero_offset_provenance_reference":
                inputs.update({"tf_initial_sample_offset_sec": -.02,
                               "future_support_offset_sec": .02,
                               "require_reference_between_tf_samples": True})
            inputs["verify_reference_closure"] = True
            if name == "capture_offset_pending":
                inputs.update({"future_support_offset_sec": .04, "future_tf_delay_sec": .05,
                               "require_future_wait": True})
            if deskew:
                inputs["time_increment"] = 1 / 16384
            arm = run_arm(SimpleNamespace(executable=args.executable,
                expected_prefix=args.expected_prefix, parameters=path, output=output), "cuda", inputs)
            text = (output / "cuda-node.log").read_text()
            diagnostics = [dict(re.findall(r"(\w+)=([^\s,]+)", line))
                           for line in text.splitlines() if "AMCL filter " in line]
            require(len(diagnostics) == 12, f"{name}: missing source-stamped diagnostics")
            require(all(row["status"] == "ok" for row in diagnostics),
                    f"{name}: unexpected rejected observation")
            require(all(float(row["prior_ess"]) >= 1 and
                        0 < float(row["posterior_max_weight"]) <= 1 and
                        float(row["largest_cluster_mass"]) > 0 and
                        row["diversity"] in ("single_parent", "posterior_point_mass", "nondegenerate_weights")
                        for row in diagnostics), f"{name}: missing pre-observation/diversity statistics")
            require("clock_evidence=uncalibrated" in text and
                    "effective_source_indices=" in text and "excluded_source_reasons=" in text,
                    f"{name}: missing beam-selection or physical-evidence provenance")
            for row in arm["rows"]:
                matrix = np.array(row["covariance"]).reshape(6, 6)[np.ix_([0, 1, 5], [0, 1, 5])]
                require(np.max(np.abs(matrix - matrix.T)) < 1e-12, "asymmetric covariance")
                require(np.min(np.linalg.eigvalsh(matrix)) >= -1e-12, "non-PSD covariance")
            if name == "ess_skip":
                require(all(row["resampled"] == "0" for row in diagnostics), "ESS gate did not skip")
                require(arm["rows"][0]["pose"] != arm["rows"][-1]["pose"],
                        "skipping resampling left published statistics stale")
            if deskew or name == "capture_offset_pending":
                require(all(row["stamp_ns"] > row["raw_stamp_ns"] for row in arm["rows"]),
                        "physical reference time did not reach the published pose")
                require("scan rejected" not in text, "valid delayed TF fixture lost scans")
            require(all(row["tf_pose_position_closure_m"] < 1e-8 and
                        row["tf_pose_yaw_closure_rad"] < 1e-8 for row in arm["rows"]),
                    f"{name}: map->odom and published reference pose disagree")
            if inputs.get("require_future_wait"):
                require(all(row["future_tf_wait_verified"] for row in arm["rows"]),
                        f"{name}: required future TF was not withheld before processing")
            if name in ("deskew_curve_reference", "zero_offset_provenance_reference"):
                require(min(row["linear_reference_position_gap_m"] for row in arm["rows"]) > 1e-4,
                        "curve fixture cannot distinguish SE(2) from linear translation interpolation")
            if name == "zero_offset_provenance_reference":
                require(all(row["stamp_ns"] == row["raw_stamp_ns"] and
                            row["reference_between_tf_samples"] for row in arm["rows"]),
                        "zero-offset provenance case did not use a raw-stamp interpolation reference")
                require("mapping_id=synthetic-zero-offset-calibrated-v1" in text,
                        "custom zero-offset provenance missing from actual-node diagnostics")
                require("scan rejected" not in text, "zero-offset interpolation fixture lost scans")
            report["scenarios"][name] = {"frames": len(arm["rows"]),
                "reference_between_tf_samples": all(row.get("reference_between_tf_samples", False) for row in arm["rows"]),
                "maximum_tf_pose_position_closure_m": max(row["tf_pose_position_closure_m"] for row in arm["rows"]),
                "maximum_tf_pose_yaw_closure_rad": max(row["tf_pose_yaw_closure_rad"] for row in arm["rows"]),
                "minimum_linear_reference_gap_m": min(row["linear_reference_position_gap_m"] for row in arm["rows"]),
                "future_tf_wait_verified": all(row.get("future_tf_wait_verified", False) for row in arm["rows"]),
                "diagnostics": diagnostics, "mapped_libraries": arm["mapped_libraries"],
                "observation_counters": arm["observation_counters"], "status": "passed"}
        rejected = {}
        for name, overrides, reason in [
            ("beta_zero", {"likelihood_temperature_beta": 0.}, "AMCL beta"),
            ("ess_above_one", {"resample_ess_ratio": 1.1}, "ESS ratio"),
            ("recovery_fast_above_one", {"recovery_alpha_fast": 1.1}, "recovery"),
            ("recovery_slow_negative", {"recovery_alpha_slow": -.1}, "recovery"),
            ("confidence_one", {"kld_confidence_probability": 1.}, "kld_confidence_probability"),
            ("confidence_legacy_conflict", {"pf_z": .99, "kld_confidence_probability": .95}, "mutually exclusive"),
        ]:
            parameters = {**base, **overrides}
            if name == "confidence_one":
                parameters.pop("pf_z", None)
            rejected[name] = reject_configuration(args, parameters, args.output / ("invalid_" + name), reason)
        report["rejected_configurations"] = rejected
        report["status"] = "passed"
    except Exception as error:
        report["status"], report["error"] = "failed", str(error)
        raise
    finally:
        (args.output / "validation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "scenarios": list(report["scenarios"])}))


if __name__ == "__main__":
    main()
