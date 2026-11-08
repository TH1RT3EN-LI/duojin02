#!/usr/bin/env python3
"""Paired actual AMCL-node replay on a frozen synthetic occupancy-grid fixture.

Run inside the ROS/CUDA environment after selecting the new install prefix.
The map, odometry and scan fields are identical across arms. Ground truth remains
in this evaluator. This is a deterministic synthetic localization regression,
not a physical sensor benchmark or closed-loop navigation trial.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import yaml


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def quantiles(values):
    values = sorted(values)
    require(bool(values), "No timings were captured")
    return {"count": len(values), **{
        name: values[min(len(values) - 1, math.ceil(q * len(values)) - 1)]
        for name, q in (("p50_ms", .5), ("p95_ms", .95), ("p99_ms", .99), ("max_ms", 1))}}


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def compose_pose(a, b):
    c, sn = math.cos(a[2]), math.sin(a[2])
    return [a[0] + c * b[0] - sn * b[1], a[1] + sn * b[0] + c * b[1],
            angle_delta(a[2] + b[2], 0.)]


def integrate_body_twist(pose, twist, duration):
    """Exact SE(2) exponential, used as an independent analytic fixture oracle."""
    vx, vy, omega = twist
    if abs(omega) < 1e-12:
        relative = [duration * vx, duration * vy, duration * omega]
    else:
        angle = duration * omega
        a, b = math.sin(angle) / omega, (1. - math.cos(angle)) / omega
        relative = [a * vx - b * vy, b * vx + a * vy, angle]
    return compose_pose(pose, relative)


def grid_raycast(grid, x, y, angle):
    width, height, resolution = grid["width"], grid["height"], grid["resolution"]
    ox, oy = grid["origin"]
    cells = grid["data"]
    dx, dy = math.cos(angle), math.sin(angle)
    ix, iy = math.floor((x - ox) / resolution), math.floor((y - oy) / resolution)
    sx, sy = (1 if dx >= 0 else -1), (1 if dy >= 0 else -1)
    tx = ((ox + (ix + (sx > 0)) * resolution - x) / dx
          if abs(dx) > 1e-12 else math.inf)
    ty = ((oy + (iy + (sy > 0)) * resolution - y) / dy
          if abs(dy) > 1e-12 else math.inf)
    stepx, stepy = resolution / max(abs(dx), 1e-12), resolution / max(abs(dy), 1e-12)
    distance = 0.
    while 0 <= ix < width and 0 <= iy < height and distance < 8.:
        if cells[iy * width + ix] == 100:
            return min(8., distance)
        if tx < ty:
            ix, distance, tx = ix + sx, tx, tx + stepx
        else:
            iy, distance, ty = iy + sy, ty, ty + stepy
    return 8.


def fixture(count):
    width, height, resolution, ox, oy = 240, 200, .05, -6., -5.
    # Occupied cells are defined by their centres. Scans raycast their complete
    # square extents; the native AMCL map conversion is not modified or aligned.
    rectangles = [(-5.0, -4., 5., -3.8), (-5., 3.8, 5., 4.),
                  (-5., -4., -4.8, 4.), (4.8, -4., 5., 4.),
                  (2.5, 1.5, 3., 2.8), (-2.8, -.6, -2.2, .4),
                  (-.7, -3., .5, -2.5)]
    cells = [100 if any(x0 <= ox + (x + .5) * resolution <= x1 and
                       y0 <= oy + (y + .5) * resolution <= y1
                       for x0, y0, x1, y1 in rectangles) else 0
             for y in range(height) for x in range(width)]

    grid = {"width": width, "height": height, "resolution": resolution,
            "origin": [ox, oy], "data": cells}

    def raycast(x, y, angle):
        return grid_raycast(grid, x, y, angle)

    frames = []
    for index in range(count):
        fraction = index / max(1, count - 1)
        x, y, yaw = 1.5 * fraction, .25 * math.sin(math.pi * fraction), .20 * math.sin(2 * math.pi * fraction)
        ranges = [raycast(x, y, yaw - math.pi + beam * 2 * math.pi / 540)
                  for beam in range(540)]
        # Small deterministic perturbations exercise native scan preprocessing
        # without changing the input across arms.
        ranges = [min(8., r + .00025 * math.sin(index * 3 + beam)) for beam, r in enumerate(ranges)]
        frames.append({"stamp_ns": 100_000_000_000 + index * 100_000_000,
                       "truth": [x, y, yaw],
                       "odometry": [1.01 * x, y + .01 * fraction, yaw + .005 * fraction],
                       "ranges": ranges})
    return {"map": {"width": width, "height": height, "resolution": resolution,
                    "origin": [ox, oy], "data": cells}, "frames": frames,
            "geometry": "occupied grid-cell extents; no fitted AMCL/map alignment"}


def rolling_curve_fixture(count, rolling=True):
    """Self-consistent rolling scans on constant body twist, including a lever arm.

    TF knots bracket each scan; the midpoint is never itself published as TF.
    Set rolling=False for instantaneous D0 scans with bracketing TF knots.
    The high angular rate deliberately makes SE(2) and tf2 linear translations
    differ by millimetres. This is an interpolation-contract stress fixture.
    """
    inputs = fixture(1)
    twist, increment, offset = [2., 0., 10.], (1 / 16384 if rolling else 0.), [.3, .05, .1]
    # Match the float32 LaserScan angles and integer-nanosecond beam times
    # actually delivered over ROS, rather than raycasting unencoded doubles.
    import struct
    f32 = lambda value: struct.unpack("f", struct.pack("f", value))[0]
    angle_min, angle_increment = f32(-math.pi), f32(2 * math.pi / 540)
    frames = []
    for index in range(count):
        start = integrate_body_twist([0., 0., 0.], twist, .1 * index)
        ranges = []
        for beam in range(540):
            base = integrate_body_twist(start, twist, round(beam * increment * 1e9) / 1e9)
            laser = compose_pose(base, offset)
            ranges.append(grid_raycast(inputs["map"], laser[0], laser[1],
                                       laser[2] + angle_min + beam * angle_increment))
        frames.append({"stamp_ns": 100_000_000_000 + index * 100_000_000,
                       "truth": start, "odometry": start, "ranges": ranges})
    inputs.update({"frames": frames, "time_increment": increment, "laser_offset": offset,
                   "curve_end_transform": twist, "verify_reference_closure": True,
                   "require_future_wait": True,
                   "geometry": "constant body twist rolling rays; encoded float32 angles/nanosecond beam times; no truth fitting"})
    return inputs


def run_arm(args, backend, inputs):
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
    from tf2_msgs.msg import TFMessage

    parameters = yaml.safe_load(args.parameters.read_text())["amcl"]["ros__parameters"]
    parameters.update({"use_sim_time": True, "observation_backend": backend,
                       "observation_cuda_min_particles": 0, "observation_log_every": 1,
                       "set_initial_pose": False, "local_initialization_enabled": False})
    parameter_path = args.output / f"{backend}-parameters.yaml"
    parameter_path.write_text(yaml.safe_dump({"amcl": {"ros__parameters": parameters}}))
    logfile = args.output / f"{backend}-node.log"
    rclpy.init(args=[])
    node = Node("cuda_amcl_replay_observer")
    received, map_odom_transforms = {}, {}

    def on_tf(message):
        for transform in message.transforms:
            if transform.header.frame_id.lstrip("/") != "map" or transform.child_frame_id.lstrip("/") != "odom":
                continue
            ns = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
            p, q = transform.transform.translation, transform.transform.rotation
            map_odom_transforms[ns] = [p.x, p.y, 2 * math.atan2(q.z, q.w)]

    node.create_subscription(TFMessage, "/tf", on_tf, qos_profile_sensor_data)

    def on_pose(message):
        ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        p, q = message.pose.pose.position, message.pose.pose.orientation
        received[ns] = {"received_monotonic_ns": time.monotonic_ns(),
                        "pose": [p.x, p.y, 2 * math.atan2(q.z, q.w)],
                        "covariance": list(message.pose.covariance)}

    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_pose, 100)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(OccupancyGrid, "/map", durable)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    scan_pub = node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    tf, static_tf = TransformBroadcaster(node), StaticTransformBroadcaster(node)
    lifecycle = node.create_client(ChangeState, "/amcl/change_state")
    message_hash = hashlib.sha256()
    cdr_hash = hashlib.sha256()

    def record_input(message):
        # CDR contains alignment padding whose unspecified bytes can differ
        # between serializations. Compare every ROS message field, while also
        # preserving the raw serializer digest as supplemental evidence.
        message_hash.update(json.dumps(message_to_ordereddict(message), sort_keys=True,
                                       separators=(",", ":")).encode())
        cdr_hash.update(serialize_message(message))

    def stamp(ns):
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def spin(duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=min(.01, max(0., until - time.monotonic())))

    def state(transition):
        request = ChangeState.Request()
        request.transition.id = transition
        future = lifecycle.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.)
        require(future.done() and future.result() and future.result().success,
                f"{backend}: lifecycle transition {transition} failed")

    rows, mapped_libraries, returncode = [], [], None
    with logfile.open("w") as log:
        process = subprocess.Popen([str(args.executable), "--ros-args", "--params-file", str(parameter_path)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            require(lifecycle.wait_for_service(timeout_sec=15.), f"{backend}: AMCL lifecycle unavailable")
            clock_pub.publish(Clock(clock=stamp(inputs["frames"][0]["stamp_ns"])))
            state(Transition.TRANSITION_CONFIGURE)
            grid = OccupancyGrid()
            grid.header.frame_id = "map"
            grid.header.stamp = stamp(0)
            grid.info.width, grid.info.height = inputs["map"]["width"], inputs["map"]["height"]
            grid.info.resolution = inputs["map"]["resolution"]
            grid.info.origin.position.x, grid.info.origin.position.y = inputs["map"]["origin"]
            grid.info.origin.orientation.w = 1.
            grid.data = inputs["map"]["data"]
            record_input(grid)
            map_pub.publish(grid)
            static = TransformStamped()
            static.header.frame_id, static.child_frame_id = "base_footprint", "laser"
            laser_offset = inputs.get("laser_offset", [0., 0., 0.])
            static.transform.translation.x, static.transform.translation.y = laser_offset[:2]
            static.transform.rotation.z = math.sin(laser_offset[2] / 2.)
            static.transform.rotation.w = math.cos(laser_offset[2] / 2.)
            static_tf.sendTransform(static)
            spin(.5)
            state(Transition.TRANSITION_ACTIVATE)
            initial = PoseWithCovarianceStamped()
            initial.header.frame_id, initial.header.stamp = "map", stamp(0)
            initial.pose.pose.position.x, initial.pose.pose.position.y = .04, -.02
            initial.pose.pose.orientation.z, initial.pose.pose.orientation.w = math.sin(.015), math.cos(.015)
            initial.pose.covariance[0] = initial.pose.covariance[7] = .0004
            initial.pose.covariance[35] = .0004
            record_input(initial)
            until = time.monotonic() + 5
            while initial_pub.get_subscription_count() == 0 and time.monotonic() < until:
                spin(.05)
            require(initial_pub.get_subscription_count() > 0, "AMCL initial-pose subscriber unavailable")
            initial_pub.publish(initial)
            spin(.3)
            for frame in inputs["frames"]:
                ns = frame["stamp_ns"]
                clock = Clock(clock=stamp(ns))
                clock_pub.publish(clock)
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = "odom", "base_footprint"
                initial_sample_offset_ns = round(inputs.get("tf_initial_sample_offset_sec", 0.) * 1e9)
                require(initial_sample_offset_ns <= 0, "initial TF sample must not follow the raw scan")
                initial_tf_ns = ns + initial_sample_offset_ns
                transform.header.stamp = stamp(initial_tf_ns)
                x, y, yaw = frame["odometry"]
                initial_tf_pose = list(frame["odometry"])
                if inputs.get("curve_end_transform"):
                    initial_tf_pose = integrate_body_twist(initial_tf_pose, inputs["curve_end_transform"],
                                                          initial_sample_offset_ns / 1e9)
                transform.transform.translation.x, transform.transform.translation.y = initial_tf_pose[:2]
                transform.transform.rotation.z = math.sin(initial_tf_pose[2] / 2)
                transform.transform.rotation.w = math.cos(initial_tf_pose[2] / 2)
                tf.sendTransform(transform)
                odometry = Odometry()
                odometry.header.frame_id, odometry.child_frame_id, odometry.header.stamp = "odom", "base_footprint", stamp(ns)
                odometry.pose.pose.position.x, odometry.pose.pose.position.y = x, y
                odometry.pose.pose.orientation.z = math.sin(yaw / 2)
                odometry.pose.pose.orientation.w = math.cos(yaw / 2)
                odom_pub.publish(odometry)
                spin(.03)
                scan = LaserScan()
                scan.header.frame_id, scan.header.stamp = "laser", stamp(ns)
                scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / 540
                scan.angle_max = scan.angle_min + 539 * scan.angle_increment
                scan.range_min, scan.range_max, scan.scan_time = .3, 8., .1
                scan.time_increment = inputs.get("time_increment", 0.)
                scan.ranges = frame["ranges"]
                for message in (clock, transform, odometry, scan):
                    record_input(message)
                before = time.monotonic_ns()
                scan_pub.publish(scan)
                reference_ns = ns + round(parameters.get("capture_time_offset_sec", 0.) * 1e9) + round(
                    parameters.get("scan_reference_fraction", 0.) * (len(scan.ranges) - 1) *
                    scan.time_increment * 1e9)
                support_offset_ns = max(round((len(scan.ranges) - 1) * scan.time_increment * 1e9),
                    round(inputs.get("future_support_offset_sec", 0.) * 1e9))
                reference_pose = list(frame["odometry"])
                linear_reference_pose = list(reference_pose)
                future_sent_ns = None
                if support_offset_ns > 0:
                    # The future support sample is deliberately delivered after the
                    # scan, exercising queue retention rather than extrapolation.
                    spin(inputs.get("future_tf_delay_sec", .03))
                    if inputs.get("require_future_wait"):
                        require(reference_ns not in received,
                                f"{backend}: published a reference pose before required future TF")
                    from copy import deepcopy
                    end_transform = deepcopy(transform)
                    end_transform.header.stamp = stamp(ns + support_offset_ns)
                    end_pose = list(frame["odometry"])
                    if inputs.get("curve_end_transform"):
                        twist = inputs["curve_end_transform"]
                        end_pose = integrate_body_twist(frame["odometry"], twist, support_offset_ns / 1e9)
                        reference_pose = integrate_body_twist(frame["odometry"], twist, (reference_ns - ns) / 1e9)
                        u = (reference_ns - initial_tf_ns) / (ns + support_offset_ns - initial_tf_ns)
                        linear_reference_pose = [initial_tf_pose[j] + u *
                            (end_pose[j] - initial_tf_pose[j]) for j in (0, 1)] + [reference_pose[2]]
                    end_transform.transform.translation.x, end_transform.transform.translation.y = end_pose[:2]
                    end_transform.transform.rotation.z = math.sin(end_pose[2] / 2)
                    end_transform.transform.rotation.w = math.cos(end_pose[2] / 2)
                    record_input(end_transform)
                    future_sent_ns = time.monotonic_ns()
                    tf.sendTransform(end_transform)
                verify_closure = inputs.get("verify_reference_closure", False)
                expected_tf_ns = reference_ns + round(parameters.get("transform_tolerance", 1.) * 1e9)
                until = time.monotonic() + 3.
                while (reference_ns not in received or
                       (verify_closure and expected_tf_ns not in map_odom_transforms)) and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.01)
                    require(process.poll() is None, f"{backend}: AMCL exited during replay")
                require(reference_ns in received, f"{backend}: no estimate for reference timestamp {reference_ns}; inspect {logfile}")
                row = {"stamp_ns": reference_ns, "raw_stamp_ns": ns,
                       "truth": frame["truth"], "odometry": frame["odometry"], **received[reference_ns]}
                if inputs.get("curve_end_transform"):
                    row["truth"] = integrate_body_twist(frame["truth"], inputs["curve_end_transform"],
                                                        (reference_ns - ns) / 1e9)
                if inputs.get("require_future_wait"):
                    require(future_sent_ns is not None, "future-wait fixture has no future support sample")
                    require(row["received_monotonic_ns"] >= future_sent_ns,
                            "reference pose predates required future support delivery")
                    row["future_tf_wait_verified"] = True
                    row["future_tf_sent_monotonic_ns"] = future_sent_ns
                if inputs.get("require_reference_between_tf_samples"):
                    require(initial_tf_ns < reference_ns < ns + support_offset_ns,
                            "reference is not strictly inside its TF sample bracket")
                    row["reference_between_tf_samples"] = True
                    row["odom_tf_sample_bracket_ns"] = [initial_tf_ns, ns + support_offset_ns]
                if verify_closure:
                    require(expected_tf_ns in map_odom_transforms,
                            f"{backend}: no map->odom TF for reference output {reference_ns}")
                    map_odom = map_odom_transforms[expected_tf_ns]
                    closed_pose = compose_pose(map_odom, reference_pose)
                    position_gap = math.hypot(closed_pose[0] - row["pose"][0], closed_pose[1] - row["pose"][1])
                    yaw_gap = abs(angle_delta(closed_pose[2], row["pose"][2]))
                    require(position_gap < 1e-8 and yaw_gap < 1e-8,
                            f"{backend}: map->odom/reference pose contract mismatch: {position_gap} m, {yaw_gap} rad")
                    row.update({"map_odom_tf_stamp_ns": expected_tf_ns, "map_odom_pose": map_odom,
                                "odom_base_reference": reference_pose,
                                "tf_pose_position_closure_m": position_gap, "tf_pose_yaw_closure_rad": yaw_gap,
                                "linear_reference_position_gap_m": math.hypot(
                                    reference_pose[0] - linear_reference_pose[0],
                                    reference_pose[1] - linear_reference_pose[1])})
                row["scan_publish_to_pose_receipt_ms"] = (row["received_monotonic_ns"] - before) / 1e6
                row["position_error_m"] = math.hypot(row["pose"][0] - row["truth"][0], row["pose"][1] - row["truth"][1])
                row["yaw_error_rad"] = abs(angle_delta(row["pose"][2], row["truth"][2]))
                require(all(math.isfinite(value) for value in row["pose"] + row["covariance"]), "Nonfinite AMCL output")
                rows.append(row)
            paths = {line.split()[-1] for line in Path(f"/proc/{process.pid}/maps").read_text().splitlines()
                     if "/" in line and ("amcl" in line.lower() or "cuda" in line.lower())}
            mapped_libraries = sorted(paths)
            relevant = [path for path in paths if "amcl" in path.lower() and ".so" in path]
            require(relevant, "No AMCL shared library mapped by actual node")
            require(all(path.startswith(str(args.expected_prefix) + "/") for path in relevant),
                    f"AMCL loaded outside expected prefix: {relevant}")
            state(Transition.TRANSITION_DEACTIVATE)
            state(Transition.TRANSITION_CLEANUP)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                returncode = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                returncode = process.wait(timeout=3)
            node.destroy_node()
            rclpy.shutdown()
            (args.output / f"{backend}-rows.json").write_text(json.dumps(rows, indent=2) + "\n")
    require(returncode == 0, f"{backend}: abnormal AMCL shutdown {returncode}")
    text = logfile.read_text()
    counters = [dict(re.findall(r"(\w+)=([^\s,]+)", line))
                for line in text.splitlines() if "AMCL_OBSERVATION_STATS" in line]
    if backend == "cuda":
        require(counters and max(int(row.get("cuda_updates", 0)) for row in counters) > 0,
                "Actual-node replay did not verify a CUDA observation update")
        require(max(int(row.get("cuda_updates", 0)) + int(row.get("cpu_updates", 0))
                    for row in counters) >= len(rows), "Missing observation updates in CUDA arm")
        require(all(int(row.get("cpu_updates", 0)) == int(row.get("boundary_fallbacks", 0))
                    for row in counters), "CUDA arm used CPU without a recorded boundary fallback")
        require(max(int(row.get("failures", 0)) for row in counters) == 0, "CUDA observation failures occurred")
    return {"backend": backend, "parameters": parameters, "rows": rows, "mapped_libraries": mapped_libraries,
            "input_messages_sha256": message_hash.hexdigest(), "observation_counters": counters,
            "input_message_cdr_sha256": cdr_hash.hexdigest(),
            "scan_publish_to_pose_receipt": quantiles([row["scan_publish_to_pose_receipt_ms"] for row in rows]),
            "position_rmse_m": math.sqrt(sum(row["position_error_m"] ** 2 for row in rows) / len(rows)),
            "position_max_m": max(row["position_error_m"] for row in rows),
            "yaw_rmse_rad": math.sqrt(sum(row["yaw_error_rad"] ** 2 for row in rows) / len(rows)),
            "returncode": returncode}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--expected-prefix", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--maximum-position-delta", type=float, default=.03)
    parser.add_argument("--maximum-yaw-delta", type=float, default=.03)
    args = parser.parse_args()
    require(args.frames >= 10, "At least 10 replay frames are required")
    require(not (args.output / "replay.json").exists(), "Preserve old evidence: select a new output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    from ros_isolation import configure_isolation
    domain = configure_isolation()
    inputs = fixture(args.frames)
    raw_fixture = json.dumps(inputs, sort_keys=True, separators=(",", ":"))
    (args.output / "fixture.json").write_text(raw_fixture + "\n")
    report = {"schema_version": 1, "status": "running", "scope": "actual_amcl_node_synthetic_paired_replay",
              "ros_domain_id": domain, "fixture_sha256": hashlib.sha256(raw_fixture.encode()).hexdigest(),
              "frames": args.frames, "arms": [],
              "timing_scope": "scan publish call to matching source-stamp pose receipt; includes DDS, TF filtering, PF update and scheduling",
              "limitations": ["Synthetic grid raycasts and biased kinematic odometry; not recorded physical scans",
                              "Fresh processes may amplify floating-point differences through adaptive resampling",
                              "Map-cell conventions retained without truth fitting; no physical or Jetson timing claim"]}
    report_path = args.output / "replay.json"
    try:
        for backend in ("cpu", "cuda"):
            report["arms"].append(run_arm(args, backend, inputs))
            report_path.write_text(json.dumps(report, indent=2) + "\n")
        cpu, gpu = report["arms"]
        require(cpu["input_messages_sha256"] == gpu["input_messages_sha256"], "Replay input messages differ across arms")
        report["input_hash_scope"] = "Canonical complete ROS message fields; CDR serializer padding is excluded"
        report["raw_cdr_hashes_equal"] = cpu["input_message_cdr_sha256"] == gpu["input_message_cdr_sha256"]
        differences = []
        for a, b in zip(cpu["rows"], gpu["rows"]):
            require(a["stamp_ns"] == b["stamp_ns"], "CPU/GPU source stamps do not match")
            differences.append({"stamp_ns": a["stamp_ns"],
                                "position_m": math.hypot(a["pose"][0] - b["pose"][0], a["pose"][1] - b["pose"][1]),
                                "yaw_rad": abs(angle_delta(a["pose"][2], b["pose"][2])),
                                "maximum_covariance_delta": max(abs(x - y) for x, y in zip(a["covariance"], b["covariance"]))})
        report["paired_differences"] = differences
        report["maximum_position_delta_m"] = max(row["position_m"] for row in differences)
        report["maximum_yaw_delta_rad"] = max(row["yaw_rad"] for row in differences)
        report["acceptance"] = {"maximum_position_delta_m": args.maximum_position_delta,
                                "maximum_yaw_delta_rad": args.maximum_yaw_delta,
                                "minimum_source_stamp_coverage": 1.0}
        require(report["maximum_position_delta_m"] <= args.maximum_position_delta, "CPU/GPU position divergence exceeds declared tolerance")
        require(report["maximum_yaw_delta_rad"] <= args.maximum_yaw_delta, "CPU/GPU yaw divergence exceeds declared tolerance")
        report["status"] = "passed"
    except Exception as error:
        report["status"], report["error"] = "failed", str(error)
        raise
    finally:
        report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "frames": args.frames,
                      "maximum_position_delta_m": report["maximum_position_delta_m"],
                      "maximum_yaw_delta_rad": report["maximum_yaw_delta_rad"],
                      "arm_timings": {arm["backend"]: arm["scan_publish_to_pose_receipt"]
                                      for arm in report["arms"]}}, indent=2))


if __name__ == "__main__":
    main()
