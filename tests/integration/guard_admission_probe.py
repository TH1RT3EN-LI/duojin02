#!/usr/bin/env python3
"""Exercise exact-time TF admission and invalid odometry through a real guard."""
import argparse
import json
import math
import os
from pathlib import Path
import random
import time
import types

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from tf2_msgs.msg import TFMessage

from scan_fault_probe import CaseProbe


def exercise(args, name, lag, odometry_fault=None, map_tf=False):
    probe = CaseProbe(name, args)
    result = {
        "case": name,
        "tf_lag_s": lag,
        "tf_edge": "odom_map" if map_tf else "odom_laser",
        "passed": False,
    }
    try:
        probe.wait_ready()
        tf_publisher = probe.create_publisher(TFMessage, probe.topic("unused_tf"), 10)
        if odometry_fault:

            def keepalive(self):
                if time.monotonic() < self.next_keepalive:
                    return
                self.next_keepalive = time.monotonic() + 0.02
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id, odom.child_frame_id = "odom", "base_footprint"

                if odometry_fault == "zero_quaternion":
                    odom.pose.pose.orientation.w = 0.0
                elif odometry_fault == "invalid_nanosecond":
                    odom.header.stamp.nanosec = 1_000_000_000
                self.odom_pub.publish(odom)
                command = Twist()
                command.linear.x = 0.3
                self.command_pub.publish(command)

            probe.keepalive = types.MethodType(keepalive, probe)

        started = time.monotonic()
        next_input = 0.0
        while time.monotonic() - started < 2.0:
            if time.monotonic() >= next_input:
                scan = probe.scan([math.inf] * 33)
                scan.header.frame_id = "odom" if map_tf else "laser"
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = "odom", (
                    "map" if map_tf else "laser"
                )
                transform.transform.rotation.w = 1.0

                warmup = map_tf and time.monotonic() - started < 0.4
                transform.header.stamp = (
                    rclpy.time.Time.from_msg(scan.header.stamp)
                    - Duration(seconds=0.0 if warmup else lag)
                ).to_msg()
                tf_publisher.publish(TFMessage(transforms=[transform]))
                if warmup:
                    probe.pump(0.006)
                probe.scan_pub.publish(scan)
                next_input = time.monotonic() + 0.05
            probe.spin()

        statuses = [value for stamp, value in probe.statuses if stamp >= started + 1.0]
        outputs = [value for stamp, value in probe.outputs if stamp >= started + 1.0]
        assert statuses and outputs, "No guard output during the steady observation window"
        result.update(
            statuses=len(statuses),
            nonzero_outputs=sum(value["linear_x"] > 0.01 for value in outputs),
            reasons={
                reason: sum(value["reason"] == reason for value in statuses)
                for reason in {value["reason"] for value in statuses}
            },
            tf_ready_scans=statuses[-1]["tf_ready_scans"],
        )
        if odometry_fault or lag > 0.3:
            assert not result["nonzero_outputs"], result
        else:
            assert result["nonzero_outputs"] == len(outputs), result
            assert set(result["reasons"]) == {"clear"}, result
        result["passed"] = True
    finally:
        probe.close()
        result["guard_exit"] = probe.process.returncode
        result["passed"] &= probe.process.returncode == 0
        (args.output_dir / (name + ".json")).write_text(json.dumps(result, indent=2) + "\n")
    assert result["passed"], result
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--guard-binary", type=Path, required=True)
    parser.add_argument("--params-file", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.log_dir, args.discovery_timeout_s = args.output_dir, 8.0
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(80, 220))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    results = []
    rclpy.init(args=[])
    try:
        for name, lag, fault, map_tf in (
            ("tf_0ms", 0.0, None, False),
            ("tf_100ms", 0.1, None, False),
            ("tf_200ms", 0.2, None, False),
            ("tf_400ms_expired", 0.4, None, False),
            ("zero_quaternion", 0.0, "zero_quaternion", False),
            ("invalid_nanosecond", 0.0, "invalid_nanosecond", False),
            ("map_tf_100ms", 0.1, None, True),
            ("map_tf_400ms_expired", 0.4, None, True),
        ):
            results.append(exercise(args, name, lag, fault, map_tf))
    finally:
        rclpy.try_shutdown()
    print(json.dumps({"passed": True, "cases": results}, indent=2))


if __name__ == "__main__":
    main()
