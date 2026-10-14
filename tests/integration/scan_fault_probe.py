#!/usr/bin/env python3
"""Synthetic, isolated API regression probe for VelocityGuard scan faults.

Run in an environment where rclpy and the supplied guard's ROS libraries are
available, after the guard has been built by the caller:

  python3 tests/integration/scan_fault_probe.py --guard-binary /absolute/path/velocity_guard \
      --params-file /absolute/path/safety.yaml \
      --json-out /absolute/path/scan_fault_results.json

The probe does not build anything or contact hardware. It uses a random local
ROS domain by default (override with --domain-id), and remaps every guard input
and output, including TF, to a unique synthetic topic prefix for each case.
Only stationary odometry, requested velocity, and odom-frame LaserScan messages
are supplied; there is no map, costmap, localization prior, or sensor TF input.

Each fault is sent exactly once while command and odometry stay fresh. The
default response bound is one 50 ms guard tick plus 30 ms transport/scheduling
tolerance, followed by observation until 160 ms after publication. This is
strictly shorter than the forced 300 ms scan timeout, so timing out the old
good scan cannot make the immediate-fault assertions pass. Results describe
only this invocation; no previously collected result is embedded in this file.
The fault-barrier replay case also republishes the exact pre-fault good scan
while its stamp is still fresh. It must leave the fault locked and must not
publish accepted-scan obstacle memory; only a strictly newer good stamp may
restore command flow.
The same sequence also runs with an otherwise valid scan carrying sec=-1 to
check that a malformed stamp falls back to the last accepted stamp barrier.
An independent normal_duplicate_timeout case repeats one accepted good scan
without injecting any fault. It verifies that duplicates cannot refresh the
wall heartbeat: timeout must occur from the first accepted scan, before a
guard that refreshed on the last duplicate could possibly time out. This
case assumes normal clocks; it does not pause or rewind the system clock.
"""

import argparse
import json
import math
import os
from pathlib import Path
import random
import signal
import struct
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.utilities import get_rmw_implementation_identifier
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


TICK_S = 0.050
SCAN_TIMEOUT_S = 0.300
FAULT_OBSERVATION_S = 0.160
REPLAY_OBSERVATION_S = 0.100
DUPLICATE_STREAM_END_S = 0.260
KEEPALIVE_S = 0.020
REQUESTED_X = 0.300
BEAM_COUNT = 33
RANGE_MIN = 0.300
RANGE_MAX = 8.000
FIRST_FAULTS = ("all_nan", "all_negative_inf", "empty", "all_finite_out_of_range")
METADATA_FAULTS = (
    "nan_angle_min",
    "nan_range_min",
    "nan_range_max",
    "zero_angle_increment",
    "negative_time_increment",
    "empty_frame",
)


class ProbeFailure(AssertionError):
    def __init__(self, message, **evidence):
        super().__init__(message)
        self.evidence = evidence


def require(condition, message, **evidence):
    if not condition:
        raise ProbeFailure(message, **evidence)


def zero(command):
    return all(abs(value) < 1e-9 for value in command.values())


def clear_command(command):
    return abs(command["linear_x"] - REQUESTED_X) < 1e-6 and all(
        abs(value) < 1e-9 for key, value in command.items() if key != "linear_x"
    )


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def xyz_points(cloud):

    fields = {field.name: field for field in cloud.fields}
    require(
        all(name in fields and fields[name].datatype == 7 for name in ("x", "y", "z")),
        "Obstacle-memory cloud must contain float32 XYZ fields",
    )
    endian = ">" if cloud.is_bigendian else "<"
    data = bytes(cloud.data)
    for row in range(cloud.height):
        for column in range(cloud.width):
            start = row * cloud.row_step + column * cloud.point_step
            yield tuple(
                struct.unpack_from(endian + "f", data, start + fields[name].offset)[0]
                for name in ("x", "y", "z")
            )


class CaseProbe(Node):
    def __init__(self, name, args):
        super().__init__("scan_fault_probe_" + name)
        self.case_name = name
        self.args = args
        self.prefix = "/scan_fault_probe/p" + uuid.uuid4().hex + "/" + name
        self.statuses = []
        self.outputs = []
        self.clouds = []
        self.next_keepalive = 0.0
        qos = QoSProfile(depth=10)
        self.create_subscription(String, self.topic("status"), self.on_status, qos)
        self.create_subscription(Twist, self.topic("guarded"), self.on_output, qos)
        self.create_subscription(
            PointCloud2, self.topic("memory"), self.on_cloud, qos_profile_sensor_data
        )
        self.command_pub = self.create_publisher(Twist, self.topic("requested"), qos)
        self.odom_pub = self.create_publisher(Odometry, self.topic("odom"), qos)
        self.scan_pub = self.create_publisher(
            LaserScan, self.topic("scan"), qos_profile_sensor_data
        )
        self.log_path = args.log_dir / (name + ".log")
        self.log = self.log_path.open("w", encoding="utf-8")
        remaps = {
            "/cmd_vel_requested": "requested",
            "/cmd_vel_base": "guarded",
            "/odometry/filtered": "odom",
            "/scan": "scan",
            "/navigation/guard_status": "status",
            "/navigation/obstacle_memory": "memory",
            "/map": "unused_map",
            "/local_costmap/costmap": "unused_costmap",
            "/amcl_pose": "unused_localization",
            "/tf": "unused_tf",
            "/tf_static": "unused_tf_static",
        }
        command = [
            str(args.guard_binary),
            "--ros-args",
            "--params-file",
            str(args.params_file),
            "-p",
            "use_sim_time:=false",
            "-p",
            "scan_timeout_s:=0.3",
            "-p",
            "command_timeout_s:=0.25",
        ]
        for original, suffix in remaps.items():
            command.extend(["-r", original + ":=" + self.topic(suffix)])
        try:
            self.process = subprocess.Popen(
                command,
                stdout=self.log,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
                start_new_session=True,
            )
        except BaseException:
            self.log.close()
            self.destroy_node()
            raise

    def topic(self, suffix):
        return self.prefix + "/" + suffix

    def on_status(self, message):
        self.statuses.append((time.monotonic(), json.loads(message.data)))

    def on_output(self, message):
        self.outputs.append(
            (
                time.monotonic(),
                {
                    "linear_x": message.linear.x,
                    "linear_y": message.linear.y,
                    "linear_z": message.linear.z,
                    "angular_x": message.angular.x,
                    "angular_y": message.angular.y,
                    "angular_z": message.angular.z,
                },
            )
        )

    def on_cloud(self, message):
        self.clouds.append((time.monotonic(), message))

    def snapshot(self):
        return {
            "topic_prefix": self.prefix,
            "status": self.statuses[-1][1] if self.statuses else None,
            "command": self.outputs[-1][1] if self.outputs else None,
            "cloud_points": (
                self.clouds[-1][1].width * self.clouds[-1][1].height if self.clouds else None
            ),
            "guard_log": str(self.log_path),
        }

    def check_process(self):
        require(
            self.process.poll() is None,
            "Guard exited during a probe case",
            returncode=self.process.returncode,
            **self.snapshot(),
        )

    def keepalive(self):
        current = time.monotonic()
        if current < self.next_keepalive:
            return
        self.next_keepalive = current + KEEPALIVE_S
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        command = Twist()
        command.linear.x = REQUESTED_X
        self.command_pub.publish(command)

    def spin(self):
        self.check_process()
        self.keepalive()
        rclpy.spin_once(self, timeout_sec=0.002)

    def pump(self, duration, scan_factory=None):
        deadline = time.monotonic() + max(0.0, duration)
        next_scan = 0.0
        while time.monotonic() < deadline:
            if scan_factory is not None and time.monotonic() >= next_scan:
                self.scan_pub.publish(scan_factory())
                next_scan = time.monotonic() + KEEPALIVE_S
            self.spin()

    def wait_ready(self):
        deadline = time.monotonic() + self.args.discovery_timeout_s
        while time.monotonic() < deadline:
            self.check_process()
            inputs_ready = all(
                self.count_subscribers(self.topic(suffix)) > 0
                for suffix in ("scan", "requested", "odom")
            )
            outputs_ready = all(
                self.count_publishers(self.topic(suffix)) > 0
                for suffix in ("status", "guarded", "memory")
            )

            command_ready = self.statuses and self.statuses[-1][1]["reason"] == "scan_or_tf_stale"
            if inputs_ready and outputs_ready and command_ready:
                return
            self.spin()
        raise ProbeFailure("Guard discovery timed out", **self.snapshot())

    def scan(self, ranges=None):
        message = LaserScan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        message.range_min = RANGE_MIN
        message.range_max = RANGE_MAX
        message.angle_min = -math.pi
        message.angle_increment = 2 * math.pi / (BEAM_COUNT - 1)
        message.ranges = [1.5] * BEAM_COUNT if ranges is None else ranges
        message.angle_max = message.angle_min + message.angle_increment * (len(message.ranges) - 1)
        message.scan_time = KEEPALIVE_S
        message.time_increment = KEEPALIVE_S / (BEAM_COUNT - 1)
        return message

    def fault_scan(self, name):
        ranges = {
            "all_nan": [math.nan] * BEAM_COUNT,
            "all_negative_inf": [-math.inf] * BEAM_COUNT,
            "empty": [],
            "all_finite_out_of_range": [0.1 if index % 2 else 8.5 for index in range(BEAM_COUNT)],
        }
        message = self.scan(ranges.get(name))
        if name == "nan_angle_min":
            message.angle_min = math.nan
        elif name == "nan_range_min":
            message.range_min = math.nan
        elif name == "nan_range_max":
            message.range_max = math.nan
        elif name == "zero_angle_increment":
            message.angle_increment = 0.0
        elif name == "negative_time_increment":
            message.time_increment = -0.001
        elif name == "empty_frame":
            message.header.frame_id = ""

        message.angle_max = message.angle_min + message.angle_increment * (len(message.ranges) - 1)
        return message

    def response(self, since, reason_predicate, command_predicate):
        deadline = since + TICK_S + self.args.timing_tolerance_s
        while time.monotonic() < deadline:
            self.spin()
            status = next(
                (
                    (stamp, value)
                    for stamp, value in self.statuses
                    if stamp >= since and reason_predicate(value)
                ),
                None,
            )
            output = next(
                (
                    (stamp, value)
                    for stamp, value in self.outputs
                    if stamp >= since and command_predicate(value)
                ),
                None,
            )
            if status is not None and output is not None:
                return {
                    "status_latency_ms": round((status[0] - since) * 1000, 3),
                    "command_latency_ms": round((output[0] - since) * 1000, 3),
                    "status": status[1],
                    "command": output[1],
                    "status_received_at": status[0],
                    "command_received_at": output[0],
                }
        raise ProbeFailure(
            "Expected scan response did not arrive within one tick plus tolerance",
            response_bound_ms=(TICK_S + self.args.timing_tolerance_s) * 1000,
            elapsed_ms=(time.monotonic() - since) * 1000,
            **self.snapshot(),
        )

    def invalid_once(self, name):
        self.keepalive()
        since = time.monotonic()
        self.scan_pub.publish(self.fault_scan(name))
        response = self.response(
            since, lambda value: value["reason"] == self.args.invalid_reason, zero
        )
        self.pump(FAULT_OBSERVATION_S - (time.monotonic() - since))
        require(
            self.statuses[-1][1]["reason"] == self.args.invalid_reason
            and zero(self.outputs[-1][1]),
            "Invalid scan did not hold a zero command before the old scan could time out",
            fault=name,
            observation_ms=FAULT_OBSERVATION_S * 1000,
            **self.snapshot(),
        )
        require(
            all(
                value["reason"] == self.args.invalid_reason
                for stamp, value in self.statuses
                if stamp >= response["status_received_at"]
            ),
            "Guard left scan_invalid without a new valid scan",
            fault=name,
            **self.snapshot(),
        )
        require(
            all(
                zero(value)
                for stamp, value in self.outputs
                if stamp >= response["command_received_at"]
            ),
            "A nonzero output followed the invalid-scan stop",
            fault=name,
            **self.snapshot(),
        )
        response.pop("status_received_at")
        response.pop("command_received_at")
        response["observation_ms"] = FAULT_OBSERVATION_S * 1000
        return response

    def clear_once(self):
        self.keepalive()
        since = time.monotonic()
        self.scan_pub.publish(self.scan())
        response = self.response(since, lambda value: value["reason"] == "clear", clear_command)
        self.pump(FAULT_OBSERVATION_S - (time.monotonic() - since))
        require(
            self.statuses[-1][1]["reason"] == "clear" and clear_command(self.outputs[-1][1]),
            "One valid scan did not restore normal command flow",
            **self.snapshot(),
        )
        response.pop("status_received_at")
        response.pop("command_received_at")
        return response

    def fault_case(self, name, first_fault):
        result = {}
        if first_fault:
            result["first_invalid"] = self.invalid_once(name)
            require(
                self.statuses[-1][1]["remembered_points"] == 0,
                "First invalid scan created obstacle memory",
                **self.snapshot(),
            )
        result["good_before_fault"] = self.clear_once()
        result["good_to_invalid_once"] = self.invalid_once(name)
        result["good_recovery_once"] = self.clear_once()
        return result

    def positive_inf_case(self):
        all_inf = lambda: self.scan([math.inf] * BEAM_COUNT)
        self.pump(SCAN_TIMEOUT_S + 0.150, all_inf)
        require(
            self.statuses[-1][1]["reason"] == "clear"
            and clear_command(self.outputs[-1][1])
            and self.statuses[-1][1]["remembered_points"] == 0,
            "A fresh first all-positive-Inf scan must permit a clear command",
            **self.snapshot(),
        )
        fresh_empty = self.snapshot()
        obstacle_ranges = [math.inf] * BEAM_COUNT
        obstacle_ranges[BEAM_COUNT // 2] = 0.350
        since = time.monotonic()
        self.scan_pub.publish(self.scan(obstacle_ranges))
        response = self.response(
            since,
            lambda value: value["reason"]
            in ("braking_slowdown", "swept_collision", "measured_braking_collision")
            and value["remembered_points"] == 1,
            lambda value: value["linear_x"] < REQUESTED_X - 1e-6,
        )
        self.pump(0.020)
        require(
            self.clouds and len(list(xyz_points(self.clouds[-1][1]))) == 1,
            "Finite near obstacle was not published in memory",
            **self.snapshot(),
        )
        points_before = list(xyz_points(self.clouds[-1][1]))
        since_inf = time.monotonic()
        self.pump(SCAN_TIMEOUT_S + 0.150, all_inf)
        statuses = [value for stamp, value in self.statuses if stamp >= since_inf]
        require(
            statuses
            and all(
                value["reason"] not in (self.args.invalid_reason, "scan_or_tf_stale")
                and value["remembered_points"] == 1
                for value in statuses
            ),
            "All-positive-Inf scan must stay fresh without erasing remembered obstacles",
            **self.snapshot(),
        )
        require(
            self.outputs[-1][1]["linear_x"] < REQUESTED_X - 1e-6,
            "No-return Inf erased the braking constraint from a remembered obstacle",
            **self.snapshot(),
        )
        points_after = list(xyz_points(self.clouds[-1][1]))
        require(
            points_after == points_before,
            "Inf rays changed the remembered obstacle",
            **self.snapshot(),
        )
        response.pop("status_received_at")
        response.pop("command_received_at")
        return {
            "fresh_first_all_positive_inf": fresh_empty,
            "finite_obstacle": response,
            "positive_inf_after_obstacle": self.snapshot(),
            "memory_xyz": points_after,
            "fresh_stream_duration_ms": (SCAN_TIMEOUT_S + 0.150) * 1000,
        }

    def mixed_case(self):
        ranges = [math.nan if index % 2 else math.inf for index in range(BEAM_COUNT)]
        ranges[BEAM_COUNT // 2] = 1.5
        self.pump(SCAN_TIMEOUT_S + 0.150, lambda: self.scan(ranges))
        require(
            self.statuses[-1][1]["reason"] == "clear"
            and clear_command(self.outputs[-1][1])
            and self.statuses[-1][1]["remembered_points"] == 1,
            "Mixed finite/NaN/positive-Inf scan did not retain its valid finite point",
            **self.snapshot(),
        )
        points = list(xyz_points(self.clouds[-1][1]))
        require(
            len(points) == 1
            and abs(points[0][0] - 1.5) < 1e-5
            and abs(points[0][1]) < 1e-5
            and abs(points[0][2]) < 1e-5,
            "Mixed scan's valid point has the wrong memory coordinates",
            memory_xyz=points,
            **self.snapshot(),
        )
        return {
            "fresh_mixed_scan": self.snapshot(),
            "memory_xyz": points,
            "fresh_stream_duration_ms": (SCAN_TIMEOUT_S + 0.150) * 1000,
        }

    def timeout_case(self):
        self.clear_once()
        self.pump(SCAN_TIMEOUT_S + 0.100)
        require(
            self.statuses[-1][1]["reason"] == "scan_or_tf_stale" and zero(self.outputs[-1][1]),
            "A missing scan must retain the separate timeout reason",
            **self.snapshot(),
        )
        return {"valid_scan_then_no_scan": self.snapshot(), "good_recovery_once": self.clear_once()}

    def fault_barrier_replay_case(self, malformed_stamp=False):

        good = self.scan()
        good_stamp = stamp_ns(good.header.stamp)
        initial_since = time.monotonic()
        self.scan_pub.publish(good)
        initial = self.response(
            initial_since, lambda value: value["reason"] == "clear", clear_command
        )
        deadline = initial_since + TICK_S + self.args.timing_tolerance_s
        while not any(stamp_ns(cloud.header.stamp) == good_stamp for _, cloud in self.clouds):
            require(
                time.monotonic() < deadline,
                "Initial good scan did not publish its accepted-scan memory before the replay test",
                good_stamp_ns=good_stamp,
                **self.snapshot(),
            )
            self.spin()

        fault = self.scan() if malformed_stamp else self.fault_scan("all_nan")
        if malformed_stamp:
            fault.header.stamp.sec = -1
            fault.header.stamp.nanosec = 0
        fault_stamp = stamp_ns(fault.header.stamp)
        if not malformed_stamp:
            require(
                fault_stamp > good_stamp,
                "Fault stamp must be strictly newer than the saved good stamp",
                good_stamp_ns=good_stamp,
                fault_stamp_ns=fault_stamp,
            )
        expected_barrier_stamp = good_stamp if malformed_stamp else fault_stamp
        fault_since = time.monotonic()
        self.scan_pub.publish(fault)
        invalid = self.response(
            fault_since, lambda value: value["reason"] == self.args.invalid_reason, zero
        )

        status_mark = len(self.statuses)
        output_mark = len(self.outputs)
        clouds_before = len(self.clouds)
        old_age_before = (self.get_clock().now().nanoseconds - good_stamp) / 1_000_000_000
        require(
            0 <= old_age_before < SCAN_TIMEOUT_S,
            "Saved good scan must still be fresh when replayed; timeout cannot explain its rejection",
            old_good_age_s=old_age_before,
            **self.snapshot(),
        )
        replay_since = time.monotonic()
        self.scan_pub.publish(good)
        self.pump(REPLAY_OBSERVATION_S)
        old_age_after = (self.get_clock().now().nanoseconds - good_stamp) / 1_000_000_000
        require(
            0 <= old_age_after < SCAN_TIMEOUT_S,
            "Old-good replay observation exceeded the freshness window",
            old_good_age_before_s=old_age_before,
            old_good_age_after_s=old_age_after,
            **self.snapshot(),
        )
        replay_statuses = self.statuses[status_mark:]
        replay_outputs = self.outputs[output_mark:]
        require(
            replay_statuses and replay_outputs,
            "Replay observation must include new guard ticks",
            **self.snapshot(),
        )
        require(
            all(value["reason"] == self.args.invalid_reason for _, value in replay_statuses)
            and all(zero(value) for _, value in replay_outputs),
            "Fresh but pre-fault good scan cleared the fault barrier or enabled motion",
            good_stamp_ns=good_stamp,
            fault_stamp_ns=fault_stamp,
            **self.snapshot(),
        )

        require(
            len(self.clouds) == clouds_before,
            "Old good scan crossed the acceptance barrier and refreshed scan memory",
            accepted_scan_clouds_before=clouds_before,
            accepted_scan_clouds_after=len(self.clouds),
            good_stamp_ns=good_stamp,
            fault_stamp_ns=fault_stamp,
            **self.snapshot(),
        )
        replay_snapshot = self.snapshot()
        clouds_after_replay = len(self.clouds)
        replay_elapsed_ms = round((time.monotonic() - replay_since) * 1000, 3)

        new_good = self.scan()
        new_stamp = stamp_ns(new_good.header.stamp)
        require(
            new_stamp > expected_barrier_stamp and new_stamp > good_stamp,
            "Recovery scan must have a strictly newer stamp than the effective fault barrier",
            good_stamp_ns=good_stamp,
            fault_stamp_ns=fault_stamp,
            new_good_stamp_ns=new_stamp,
        )
        recovery_since = time.monotonic()
        self.scan_pub.publish(new_good)
        recovery = self.response(
            recovery_since, lambda value: value["reason"] == "clear", clear_command
        )
        self.pump(FAULT_OBSERVATION_S - (time.monotonic() - recovery_since))
        require(
            self.statuses[-1][1]["reason"] == "clear"
            and clear_command(self.outputs[-1][1])
            and any(stamp_ns(cloud.header.stamp) == new_stamp for _, cloud in self.clouds),
            "Strictly newer good scan did not restore accepted scan memory and command flow",
            new_good_stamp_ns=new_stamp,
            **self.snapshot(),
        )
        for response in (initial, invalid, recovery):
            response.pop("status_received_at")
            response.pop("command_received_at")
        return {
            "fault_kind": "malformed_stamp_sec_minus_one" if malformed_stamp else "all_nan",
            "initial_good_once": initial,
            "invalid_once": invalid,
            "stamp_ns": {"saved_good": good_stamp, "fault": fault_stamp, "new_good": new_stamp},
            "expected_barrier_stamp_ns": expected_barrier_stamp,
            "latest_accepted_stamp_fallback": malformed_stamp,
            "old_good_replay_once": {
                "original_stamp_preserved": True,
                "age_before_s": round(old_age_before, 6),
                "age_after_s": round(old_age_after, 6),
                "observation_ms": REPLAY_OBSERVATION_S * 1000,
                "elapsed_ms": replay_elapsed_ms,
                "accepted_scan_clouds_before": clouds_before,
                "accepted_scan_clouds_after": clouds_after_replay,
                "state_after_replay": replay_snapshot,
            },
            "strictly_newer_good_recovery_once": recovery,
        }

    def normal_duplicate_timeout_case(self):
        good = self.scan()
        good_stamp = stamp_ns(good.header.stamp)
        published_at = time.monotonic()
        self.scan_pub.publish(good)
        initial = self.response(
            published_at, lambda value: value["reason"] == "clear", clear_command
        )
        acceptance_deadline = published_at + TICK_S + self.args.timing_tolerance_s
        accepted_cloud = None
        while accepted_cloud is None:
            accepted_cloud = next(
                (
                    (received, cloud)
                    for received, cloud in self.clouds
                    if stamp_ns(cloud.header.stamp) == good_stamp
                ),
                None,
            )
            if accepted_cloud is None:
                require(
                    time.monotonic() < acceptance_deadline,
                    "Initial good scan's accepted memory was not observed within the response bound",
                    good_stamp_ns=good_stamp,
                    **self.snapshot(),
                )
                self.spin()

        accepted_observed_at = accepted_cloud[0]
        clouds_before = len(self.clouds)
        duplicate_end = published_at + DUPLICATE_STREAM_END_S
        next_duplicate = time.monotonic()
        duplicate_count = 0
        last_duplicate_at = None
        last_duplicate_age_s = None
        while time.monotonic() < duplicate_end:
            if time.monotonic() >= next_duplicate:
                last_duplicate_age_s = (
                    self.get_clock().now().nanoseconds - good_stamp
                ) / 1_000_000_000
                require(
                    0 <= last_duplicate_age_s < SCAN_TIMEOUT_S,
                    "Duplicate scan must remain fresh; expiration cannot explain its rejection",
                    duplicate_age_s=last_duplicate_age_s,
                    **self.snapshot(),
                )
                last_duplicate_at = time.monotonic()
                self.scan_pub.publish(good)
                duplicate_count += 1
                next_duplicate = last_duplicate_at + KEEPALIVE_S
            self.spin()
        require(
            duplicate_count >= 2 and last_duplicate_at is not None,
            "Duplicate stream must contain multiple repeated good messages",
            duplicate_count=duplicate_count,
        )
        require(
            len(self.clouds) == clouds_before,
            "Normal duplicate scan was accepted and republished obstacle memory",
            accepted_scan_clouds_before=clouds_before,
            accepted_scan_clouds_after=len(self.clouds),
            duplicate_count=duplicate_count,
            **self.snapshot(),
        )
        require(
            self.statuses[-1][1]["reason"] == "clear" and clear_command(self.outputs[-1][1]),
            "Normal scan must remain clear before its original heartbeat times out",
            **self.snapshot(),
        )

        timeout_deadline = (
            accepted_observed_at + SCAN_TIMEOUT_S + TICK_S + self.args.timing_tolerance_s
        )
        last_duplicate_earliest_timeout = last_duplicate_at + SCAN_TIMEOUT_S
        require(
            timeout_deadline < last_duplicate_earliest_timeout,
            "Timeout observation must end before last-duplicate heartbeat could expire",
            timeout_deadline_from_publication_s=timeout_deadline - published_at,
            last_duplicate_earliest_timeout_from_publication_s=last_duplicate_earliest_timeout
            - published_at,
        )
        waiting_since = time.monotonic()
        stale_status = None
        stopped_command = None
        while time.monotonic() < timeout_deadline:
            self.spin()
            stale_status = next(
                (
                    (received, value)
                    for received, value in self.statuses
                    if received >= waiting_since and value["reason"] == "scan_or_tf_stale"
                ),
                None,
            )
            stopped_command = next(
                (
                    (received, value)
                    for received, value in self.outputs
                    if received >= waiting_since and zero(value)
                ),
                None,
            )
            if stale_status is not None and stopped_command is not None:
                break
        require(
            stale_status is not None
            and stopped_command is not None
            and stale_status[0] <= timeout_deadline
            and stopped_command[0] <= timeout_deadline,
            "Duplicates extended the heartbeat beyond the original scan's timeout plus one tick",
            timeout_bound_ms=(SCAN_TIMEOUT_S + TICK_S + self.args.timing_tolerance_s) * 1000,
            duplicate_count=duplicate_count,
            **self.snapshot(),
        )
        require(
            len(self.clouds) == clouds_before,
            "A delayed duplicate crossed the accepted scan high-water mark",
            accepted_scan_clouds_before=clouds_before,
            accepted_scan_clouds_after=len(self.clouds),
            **self.snapshot(),
        )
        clouds_after_duplicates = len(self.clouds)
        stopped_snapshot = self.snapshot()

        new_good = self.scan()
        new_stamp = stamp_ns(new_good.header.stamp)
        require(
            new_stamp > good_stamp,
            "Recovery good scan must have a strictly newer stamp",
            old_good_stamp_ns=good_stamp,
            new_good_stamp_ns=new_stamp,
        )
        recovery_since = time.monotonic()
        self.scan_pub.publish(new_good)
        recovery = self.response(
            recovery_since, lambda value: value["reason"] == "clear", clear_command
        )
        self.pump(FAULT_OBSERVATION_S - (time.monotonic() - recovery_since))
        require(
            self.statuses[-1][1]["reason"] == "clear"
            and clear_command(self.outputs[-1][1])
            and any(stamp_ns(cloud.header.stamp) == new_stamp for _, cloud in self.clouds),
            "Fresh newer scan did not recover after the duplicate timeout",
            **self.snapshot(),
        )
        for response in (initial, recovery):
            response.pop("status_received_at")
            response.pop("command_received_at")
        return {
            "initial_good_once": initial,
            "old_good_stamp_ns": good_stamp,
            "new_good_stamp_ns": new_stamp,
            "duplicate_count": duplicate_count,
            "duplicate_interval_ms": KEEPALIVE_S * 1000,
            "duplicate_stream_end_target_ms": DUPLICATE_STREAM_END_S * 1000,
            "last_duplicate_age_s": round(last_duplicate_age_s, 6),
            "accepted_memory_observed_ms": round((accepted_observed_at - published_at) * 1000, 3),
            "accepted_scan_clouds_before": clouds_before,
            "accepted_scan_clouds_after_duplicates": clouds_after_duplicates,
            "timeout_deadline_ms": round((timeout_deadline - published_at) * 1000, 3),
            "last_duplicate_earliest_timeout_ms": round(
                (last_duplicate_earliest_timeout - published_at) * 1000, 3
            ),
            "stale_status_ms": round((stale_status[0] - published_at) * 1000, 3),
            "zero_command_ms": round((stopped_command[0] - published_at) * 1000, 3),
            "stopped_after_original_heartbeat": stopped_snapshot,
            "fresh_newer_good_recovery_once": recovery,
            "clock_assumption": "Normal ROS and steady clocks; no clock pause or rewind",
        }

    def close(self):
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3)
            except ProcessLookupError:
                self.process.wait(timeout=3)
        self.log.close()
        self.destroy_node()


def arguments():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--guard-binary", type=Path, required=True)
    parser.add_argument("--params-file", type=Path, required=True)
    parser.add_argument(
        "--domain-id", type=int, default=None, help="Isolated ROS domain; default random 100..199"
    )
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path(__file__).resolve().parents[2] / ".cache/scan-fault-probe/logs",
    )
    parser.add_argument("--discovery-timeout-s", type=float, default=8.0)
    parser.add_argument("--timing-tolerance-s", type=float, default=0.030)
    parser.add_argument("--invalid-reason", default="scan_invalid")
    args = parser.parse_args()
    args.guard_binary = args.guard_binary.resolve()
    args.params_file = args.params_file.resolve()
    args.log_dir = args.log_dir.resolve()
    if not args.guard_binary.is_file() or not os.access(args.guard_binary, os.X_OK):
        parser.error("--guard-binary must be an executable file")
    if not args.params_file.is_file():
        parser.error("--params-file must be a file")
    if args.domain_id is None:
        args.domain_id = random.SystemRandom().randrange(100, 200)
    if not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be between 0 and 232")
    if args.discovery_timeout_s <= 0 or not math.isfinite(args.discovery_timeout_s):
        parser.error("--discovery-timeout-s must be finite and positive")
    if not 0 <= args.timing_tolerance_s < FAULT_OBSERVATION_S - TICK_S:
        parser.error(
            "Response tolerance must keep the response bound below the 160 ms observation window"
        )
    return args


def main():
    args = arguments()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    args.log_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "test": "VelocityGuard_scan_fault_api",
        "result": "running",
        "guard_binary": str(args.guard_binary),
        "params_file": str(args.params_file),
        "ros_domain_id": args.domain_id,
        "local_only": True,
        "scan_timeout_s": SCAN_TIMEOUT_S,
        "tick_s": TICK_S,
        "response_bound_ms": (TICK_S + args.timing_tolerance_s) * 1000,
        "invalid_observation_ms": FAULT_OBSERVATION_S * 1000,
        "boundary": "Synthetic odom-frame scan, no map/prior/TF/device/hardware input; outputs remapped",
        "cases": [],
    }
    rclpy.init(args=[])
    report["rmw_implementation"] = get_rmw_implementation_identifier()
    active_name = None
    probe = None
    exit_code = 0
    try:
        cases = [(name, "fault", True) for name in FIRST_FAULTS]
        cases += [(name, "fault", False) for name in METADATA_FAULTS]
        cases += [
            ("positive_inf", "positive_inf", False),
            ("mixed", "mixed", False),
            ("valid_scan_timeout", "timeout", False),
            ("fault_barrier_old_good_replay", "replay", False),
            ("normal_duplicate_timeout", "duplicate", False),
        ]
        for active_name, kind, first_fault in cases:
            print("Running " + active_name, file=sys.stderr, flush=True)
            probe = CaseProbe(active_name, args)
            try:
                probe.wait_ready()
                if kind == "fault":
                    details = probe.fault_case(active_name, first_fault)
                elif kind == "positive_inf":
                    details = probe.positive_inf_case()
                elif kind == "mixed":
                    details = probe.mixed_case()
                elif kind == "replay":
                    details = probe.fault_barrier_replay_case()
                    details["malformed_stamp_fallback"] = probe.fault_barrier_replay_case(
                        malformed_stamp=True
                    )
                elif kind == "duplicate":
                    details = probe.normal_duplicate_timeout_case()
                else:
                    details = probe.timeout_case()
                report["cases"].append(
                    {
                        "case": active_name,
                        "result": "passed",
                        "topic_prefix": probe.prefix,
                        "details": details,
                    }
                )
            finally:
                probe.close()
                probe = None
        report["result"] = "passed"
    except BaseException as error:
        exit_code = 130 if isinstance(error, KeyboardInterrupt) else 1
        report["result"] = "failed"
        report["failed_case"] = active_name
        report["failure"] = {
            "type": type(error).__name__,
            "message": str(error),
            "evidence": getattr(error, "evidence", {}),
        }
    finally:
        if probe is not None:
            probe.close()
        rclpy.shutdown()
    serialized = json.dumps(report, indent=2, allow_nan=False)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(serialized + "\n", encoding="utf-8")
    print(serialized)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
