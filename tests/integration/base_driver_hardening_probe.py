"""Paired real-node PTY regression; simulated clock controls only test timestamps."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

from ros_isolation import configure_isolation


def frame(tag, vx=0.3):
    data = bytearray([0x7B, 0])
    data.extend(struct.pack(">10h", round(vx * 1000), -40, 80, 10, -20, 16384, 2, -3, tag, 24000))
    checksum = 0
    for byte in data:
        checksum ^= byte
    return bytes(data + bytearray([checksum, 0x7D]))


def trace(odom, imu):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    a, w, iq = imu.linear_acceleration, imu.angular_velocity, imu.orientation
    return {
        "stamp": [imu.header.stamp.sec, imu.header.stamp.nanosec],
        "pose": [p.x, p.y, q.w, q.z],
        "twist": [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z],
        "gyro": [w.x, w.y, w.z], "accel": [a.x, a.y, a.z],
        "quaternion": [iq.w, iq.x, iq.y, iq.z],
        "gyro_covariance": list(imu.angular_velocity_covariance),
        "accel_covariance": list(imu.linear_acceleration_covariance),
        "pose_covariance": list(odom.pose.covariance),
        "twist_covariance": list(odom.twist.covariance),
        "orientation_covariance": list(imu.orientation_covariance),
    }


def probe(binary, output, hardened):
    master, slave = os.openpty()
    node = rclpy.create_node("base_driver_hardening_probe")
    odometry, inertial, diagnostics = [], [], []
    node.create_subscription(Odometry, "/odom", odometry.append, 512)
    node.create_subscription(Imu, "/imu", inertial.append, 512)
    node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics.append, 100)
    clocks = node.create_publisher(Clock, "/clock", 10)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.002)

    def clock(seconds):
        msg = Clock()
        msg.clock.sec = int(seconds)
        msg.clock.nanosec = round((seconds - int(seconds)) * 1e9)
        for _ in range(3):
            clocks.publish(msg)
            pump(0.005)

    def send(tag, stamp):
        clock(stamp)
        old = len(inertial)
        os.write(master, frame(tag))
        deadline = time.monotonic() + 0.2
        while (len(inertial) == old or len(odometry) < len(inertial)) and time.monotonic() < deadline:
            pump(0.005)
        assert len(inertial) == old + 1, (binary, tag, len(inertial), old)
        return trace(odometry[-1], inertial[-1])

    def metrics():
        pump(0.07)
        if not diagnostics:
            return {}
        return {v.key: float(v.value) for v in diagnostics[-1].status[0].values}

    arguments = [str(binary), "--ros-args", "-p", f"usart_port_name:={os.ttyname(slave)}",
                 "-p", "loop_hz:=20", "-p", "use_sim_time:=true",
                 "-p", "imu_yaw_rate_bias:=0.01", "-p", "imu_yaw_rate_variance:=0.02"]
    if hardened:
        arguments += ["-p", "feedback_queue_capacity:=16", "-p", "diagnostic_hz:=50.0"]
    result = {}
    with (output / ("candidate.log" if hardened else "baseline.log")).open("w") as log:
        process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 8
            while (node.count_publishers("/imu") == 0 or clocks.get_subscription_count() == 0) and time.monotonic() < deadline:
                pump(0.02)
            assert node.count_publishers("/imu") and clocks.get_subscription_count(), "DDS discovery failed"
            clock(10)
            deadline = time.monotonic() + 5
            while len(inertial) < 2 and time.monotonic() < deadline:
                os.write(master, frame(42))
                pump(0.06)
            assert len(inertial) >= 2, "serial receiver did not become ready"
            pump(0.08)
            odometry.clear()
            inertial.clear()
            result["normal"] = [send(100 + i, 10 + i * 0.05) for i in range(18)]
            result["burst_counts"] = []
            result["burst_tags"] = []
            for trial in range(5):
                send(200 + trial, 12 + trial)
                old = len(inertial)
                tags = list(range(300 + trial * 12, 312 + trial * 12))
                os.write(master, b"".join(frame(tag) for tag in tags))
                pump(0.15)
                actual = [round((imu.angular_velocity.z + 0.01) / 0.00026644) for imu in inertial[old:]]
                result["burst_counts"].append(len(actual))
                result["burst_tags"].append(actual)
                if hardened:
                    assert actual == tags, (tags, actual)
            send(490, 20)
            old = len(inertial)
            os.write(master, bytes([0x7B, 255, 255, 255, 255]) + frame(500))
            pump(0.13)
            result["resync_frames"] = len(inertial) - old
            # Complete the baseline's leftover partial frame without another valid header.
            os.write(master, bytes(24))
            pump(0.02)
            before = metrics()
            send(590, 21)
            old = len(inertial)
            os.write(master, b"".join(frame(600 + i) for i in range(80)))
            pump(0.14)
            after = metrics()
            result["overflow_published"] = len(inertial) - old
            result["overflow_metrics_before"] = before
            result["overflow_metrics_after"] = after
            result["time_cases"] = [send(800 + i, stamp) for i, stamp in enumerate([30, 30, 29, 32, 32.6, 32.65])]
            result["diagnostics"] = metrics()
            if hardened:
                assert result["resync_frames"] == 1
                assert after["overflow_dropped_frames"] > before["overflow_dropped_frames"]
                assert after["queue_high_watermark"] == 16
                assert after["received_frames"] == (after["published_frames"] + after["queued_frames"] + after["overflow_dropped_frames"] + after["reset_discarded_frames"])
                assert after["sequence_gaps"] > 0
                cases = result["time_cases"]
                assert all(case["orientation_covariance"][0] == -1 for case in cases[:5])
                assert cases[-1]["orientation_covariance"][0] == 1e6
                assert all(case["pose"] == cases[0]["pose"] for case in cases[1:4])
                assert all(math.isfinite(v) for case in cases for v in case["quaternion"])
            assert process.poll() is None
        finally:
            if process.poll() is None:
                os.write(master, b"".join(frame(900 + i) for i in range(128)))
                os.killpg(process.pid, signal.SIGINT)
            result["exit_code"] = process.wait(timeout=10)
            node.destroy_node()
            os.close(master)
            os.close(slave)
    assert result["exit_code"] == 0, result["exit_code"]
    return result


def invalid_parameters(binary, output):
    cases = ["imu_yaw_rate_variance:=0.0", "imu_yaw_rate_variance:=-0.01",
             "imu_yaw_rate_variance:=.nan", "imu_yaw_rate_bias:=.inf", "loop_hz:=0",
             "odom_x_scale:=.nan", "odom_twist_variances_moving:=[1.0, 2.0]",
             "odom_twist_variances_stopped:=[1.0,1.0,1.0,1.0,1.0,1.0,1.0]",
             "odom_twist_variances_moving:=[1.0,1.0,0.0,1.0,1.0,1.0]",
             "feedback_queue_capacity:=0", "diagnostic_hz:=.nan"]
    records = []
    for index, parameter in enumerate(cases):
        completed = subprocess.run([str(binary), "--ros-args", "-p", parameter],
                                   capture_output=True, timeout=4, text=True)
        (output / f"invalid-{index}.log").write_text(completed.stdout + completed.stderr)
        assert completed.returncode != 0, parameter
        assert "must be" in completed.stderr or "must contain" in completed.stderr, completed.stderr
        assert "connected" not in completed.stderr, "invalid parameters reached serial initialization"
        records.append({"parameter": parameter, "returncode": completed.returncode})
    return records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    domain = configure_isolation()
    rclpy.init()
    report = {"ros_domain_id": domain,
              "binary_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                                for name, path in [("baseline", args.baseline), ("candidate", args.candidate)]},
              "scope": "Real nodes, PTY protocol bytes and controlled host receive times; no physical devices"}
    try:
        report["baseline"] = probe(args.baseline, args.output_dir, False)
        report["candidate"] = probe(args.candidate, args.output_dir, True)
        for index, (before, after) in enumerate(zip(report["baseline"]["normal"], report["candidate"]["normal"])):
            for key in before:
                if key == "orientation_covariance" and index == 0:
                    continue
                assert before[key] == after[key], (index, key, before[key], after[key])
        assert all(count < 12 for count in report["baseline"]["burst_counts"]), "baseline loss was not reproduced"
        assert report["baseline"]["resync_frames"] == 0, "baseline parser defect was not reproduced"
        report["invalid_parameters"] = invalid_parameters(args.candidate, args.output_dir)
        report["normal_values_identical"] = True
        report["pass"] = True
    finally:
        rclpy.shutdown()
        (args.output_dir / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"pass": True, "normal_frames": 18,
                      "baseline_burst_counts": report["baseline"]["burst_counts"],
                      "candidate_burst_counts": report["candidate"]["burst_counts"],
                      "normal_values_identical": True, "invalid_parameters": len(report["invalid_parameters"])}))


if __name__ == "__main__":
    main()
