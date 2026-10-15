#!/usr/bin/env python3


import json, os, time, sys, math
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from scan_fault_probe import (
    arguments,
    CaseProbe,
    require,
    zero,
    clear_command,
    stamp_ns,
    xyz_points,
)
import rclpy


def pending_fault_case(probe, publish_tf):
    pending = probe.scan()
    pending.header.frame_id = "queued_laser"
    before = probe.statuses[-1][1]["tf_ready_scans"]
    probe.scan_pub.publish(pending)
    fault = probe.fault_scan("all_nan")
    fault.header.frame_id = pending.header.frame_id
    sent = time.monotonic()
    probe.scan_pub.publish(fault)
    response = probe.response(sent, lambda value: value["reason"] == "scan_invalid", zero)

    # The invalid callback clears queued scans before publishing its fault status.
    # Observe that status before delivering TF and explicitly replaying the old scan;
    # this exercises the stamp barrier without depending on DDS callback timing.
    publish_tf(pending)
    probe.scan_pub.publish(pending)
    probe.pump(0.070)
    rejected_count = probe.statuses[-1][1]["tf_ready_scans"]
    require(
        rejected_count == before,
        "Pre-fault scan passed the TF-ready admission barrier after the fault",
        before_tf_ready=before,
        after_replay_tf_ready=rejected_count,
    )
    require(
        probe.statuses[-1][1]["reason"] == "scan_invalid" and zero(probe.outputs[-1][1]),
        "Delayed pre-fault scan cleared the fault",
    )
    require(
        not any(stamp_ns(cloud.header.stamp) == stamp_ns(pending.header.stamp)
                for _, cloud in probe.clouds),
        "Pre-fault scan entered obstacle memory after the fault",
    )

    recovery = probe.scan()
    recovery.header.frame_id = "queued_laser"
    publish_tf(recovery)
    recovery_sent = time.monotonic()
    probe.scan_pub.publish(recovery)
    probe.response(recovery_sent, lambda value: value["reason"] == "clear", clear_command)
    recovered_count = probe.statuses[-1][1]["tf_ready_scans"]
    require(
        recovered_count > before,
        "Fresh recovery scan did not reach the TF-ready callback",
        before_tf_ready=before,
        after_recovery_tf_ready=recovered_count,
    )
    return recovery, {
        "case": "invalid_immediate_and_pending_barrier",
        "passed": True,
        "command_latency_ms": response["command_latency_ms"],
        "pending_stamp_ns": stamp_ns(pending.header.stamp),
        "fault_stamp_ns": stamp_ns(fault.header.stamp),
        "recovery_stamp_ns": stamp_ns(recovery.header.stamp),
        "tf_ready_before_fault": before,
        "tf_ready_after_old_replay": rejected_count,
        "tf_ready_after_fresh_recovery": recovered_count,
        "old_scan_replayed_after_fault_acknowledgment": True,
    }


def main():
    args = arguments()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    args.log_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=[])
    probe = None
    report = {
        "test": "guard_deferred_source_time_tf",
        "result": "running",
        "domain": args.domain_id,
        "cases": [],
    }
    try:
        probe = CaseProbe("deferred_tf", args)
        probe.wait_ready()
        tf_pub = probe.create_publisher(TFMessage, probe.topic("unused_tf"), 10)
        deadline = time.monotonic() + args.discovery_timeout_s
        while tf_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            probe.spin()
        require(tf_pub.get_subscription_count() > 0, "No remapped TF subscriber")

        def publish_tf(message, map_x=None):
            tf = TransformStamped()
            tf.header.stamp = message.header.stamp
            tf.header.frame_id = "odom"
            tf.child_frame_id = message.header.frame_id
            tf.transform.rotation.w = 1.0
            transforms = []
            if map_x is not None:
                anchor = TransformStamped()
                anchor.header.stamp = message.header.stamp
                anchor.header.frame_id = "odom"
                anchor.child_frame_id = "map"
                anchor.transform.translation.x = map_x
                anchor.transform.rotation.w = 1.0
                transforms.append(anchor)
            transforms.append(tf)
            tf_pub.publish(TFMessage(transforms=transforms))

        scan = probe.scan()
        scan.header.frame_id = "delayed_laser"
        sent = time.monotonic()
        probe.scan_pub.publish(scan)
        probe.pump(0.006)
        publish_tf(scan)
        response = probe.response(sent, lambda v: v["reason"] == "clear", clear_command)
        probe.pump(0.025)
        require(
            any(stamp_ns(c.header.stamp) == stamp_ns(scan.header.stamp) for _, c in probe.clouds),
            "Original scan stamp missing from accepted memory",
        )
        report["cases"].append(
            {
                "case": "delayed_tf_exact_stamp",
                "passed": True,
                "source_stamp_ns": stamp_ns(scan.header.stamp),
                "command_latency_ms": response["command_latency_ms"],
            }
        )

        recovery, barrier = pending_fault_case(probe, publish_tf)
        report["cases"].append(barrier)

        def anchored_cloud(offset):
            m = probe.scan([math.inf] * len(recovery.ranges))
            m.header.frame_id = "queued_laser"
            publish_tf(m, offset)
            probe.pump(0.015)
            probe.scan_pub.publish(m)
            deadline = time.monotonic() + 0.20
            while time.monotonic() < deadline:
                matches = [
                    c
                    for _, c in probe.clouds
                    if stamp_ns(c.header.stamp) == stamp_ns(m.header.stamp)
                ]
                if matches:
                    return list(xyz_points(matches[-1]))
                probe.spin()
            require(False, "Anchored source-stamp cloud was not delivered")

        before = anchored_cloud(0.0)
        after = anchored_cloud(0.10)
        require(before and len(before) == len(after), "Global correction lost remembered surfaces")
        error = max(min(math.dist((x + 0.10, y, z), p) for p in after) for x, y, z in before)
        require(
            error < 2e-6,
            "Source-time map correction did not anchor obstacle memory",
            max_error_m=error,
        )
        report["cases"].append(
            {
                "case": "global_correction_anchors_retained_memory",
                "passed": True,
                "points": len(before),
                "translation_m": 0.10,
                "max_error_m": error,
            }
        )

        start = time.monotonic()
        count = len(probe.outputs)

        def unavailable():
            m = probe.scan()
            m.header.frame_id = "missing_laser"
            return m

        probe.pump(0.400, unavailable)
        require(
            probe.statuses[-1][1]["reason"] == "scan_or_tf_stale" and zero(probe.outputs[-1][1]),
            "Missing TF refreshed the scan heartbeat",
        )
        times = [t for t, _ in probe.outputs[count:]]
        gaps = [b - a for a, b in zip(times, times[1:])]
        require(
            len(times) >= 6 and gaps and max(gaps) < 0.080,
            "TF wait blocked safety timer cadence",
            output_count=len(times),
            max_gap_ms=max(gaps) * 1000 if gaps else None,
        )
        report["cases"].append(
            {
                "case": "missing_tf_timeout_without_blocking_timer",
                "passed": True,
                "duration_s": time.monotonic() - start,
                "outputs": len(times),
                "max_gap_ms": max(gaps) * 1000,
            }
        )
        report["result"] = "passed"
    except BaseException as error:
        report["result"] = "failed"
        report["failure"] = {
            "type": type(error).__name__,
            "message": str(error),
            "evidence": getattr(error, "evidence", {}),
        }
    finally:
        if probe is not None:
            probe.close()
            report["guard_exit"] = probe.process.returncode
            errors = probe.log_path.read_text().count(
                "Do not call canTransform or lookupTransform with a timeout"
            )
            report["tf_threading_errors"] = errors
            if errors:
                report["result"] = "failed"
                report["failure"] = {
                    "type": "TfThreadingError",
                    "message": "Immediate TF queries used the blocking Buffer timeout API",
                    "count": errors,
                }
        rclpy.shutdown()
    serialized = json.dumps(report, indent=2)
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(serialized + "\n")
    print(serialized)
    return 0 if report["result"] == "passed" and report.get("guard_exit") == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
