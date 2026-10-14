"""Reconstruct a map from a bag in an isolated ROS domain and wait for queue drain."""

import argparse
import fcntl
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import yaml


def remaining(deadline, description):
    value = deadline - time.monotonic()
    if value <= 0:
        raise TimeoutError(f"{description} exceeded its timeout")
    return value


def await_response(node, future, deadline, description, processes=()):
    import rclpy
    try:
        while not future.done():
            if any(process.poll() is not None for process in processes):
                raise RuntimeError(f"Offline process stopped while waiting for {description}")
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining(deadline, description)))
        if future.exception():
            raise future.exception()
        return future.result()
    except Exception:
        future.cancel()
        raise


def inspect_bag(path, source, warmup=0.0):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import LaserScan
    from sensor_msgs.msg import Imu
    metadata = yaml.safe_load((path / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    storage = metadata["storage_identifier"]
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
                rosbag2_py.ConverterOptions("", ""))
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {"/scan": "sensor_msgs/msg/LaserScan", "/tf_static": "tf2_msgs/msg/TFMessage"}
    required.update({"/tf": "tf2_msgs/msg/TFMessage"} if source == "recorded" else {
        "/odom": "nav_msgs/msg/Odometry", "/imu": "sensor_msgs/msg/Imu"})
    for name, message_type in required.items():
        if topics.get(name) != message_type:
            raise ValueError(f"Bag requires {name} ({message_type}) for odom_source={source}")
    count = sum(entry["message_count"] for entry in metadata["topics_with_message_count"]
                if entry["topic_metadata"]["name"] == "/scan")
    if count < 2:
        raise ValueError("Bag must contain at least two laser scans")
    # TF topic presence alone does not prove that the sensor can be transformed.
    links = set()
    scan_stamps = []
    sensor_frames = set()
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        if topic == "/scan":
            message = deserialize_message(payload, LaserScan)
            stamp = message.header.stamp
            sensor_frames.add(message.header.frame_id.lstrip("/"))
            scan_stamps.append(stamp.sec * 1000000000 + stamp.nanosec)
        elif source == "ekf" and topic == "/imu":
            sensor_frames.add(deserialize_message(payload, Imu).header.frame_id.lstrip("/"))
        if topic == "/tf_static" or (source == "recorded" and topic == "/tf"):
            for transform in deserialize_message(payload, TFMessage).transforms:
                links.add((transform.header.frame_id.lstrip("/"), transform.child_frame_id.lstrip("/")))
    reachable = {"base_footprint"}
    for _ in range(len(links) + 1):
        reachable.update(child for parent, child in links if parent in reachable)
    if not sensor_frames <= reachable:
        raise ValueError(f"Bag has no base_footprint TF chain to sensors: {sorted(sensor_frames - reachable)}")
    if source == "recorded" and ("odom", "base_footprint") not in links:
        raise ValueError("Recorded odometry requires odom -> base_footprint TF")
    if len(scan_stamps) != count or any(later <= earlier for earlier, later in zip(scan_stamps, scan_stamps[1:])):
        raise ValueError("Bag scan timestamps must be strictly increasing and match its metadata")
    start = scan_stamps[0] + int(warmup * 1000000000) if source == "ekf" else 0
    skipped = sum(stamp < start for stamp in scan_stamps)
    if count - skipped < 2:
        raise ValueError("Bag has fewer than two scans after EKF warmup")
    return {"bag": str(path), "storage_id": storage, "scan_count": count - skipped,
            "recorded_scan_count": count, "warmup_skipped_scans": skipped, "scan_start_time_ns": start,
            "odom_source": source, "topics": topics}


def stop_process(process, launch=False):
    if process is None or process.poll() is not None:
        return
    try:
        # ros2 launch forwards SIGINT itself; signalling its group delivers it twice.
        if launch:
            process.send_signal(signal.SIGINT)
        else:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    except ProcessLookupError:
        # The group may exit between poll/wait and any signal. Continue cleaning
        # the remaining children and ROS resources instead of aborting finally.
        return


def run(args):
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from rosbag2_interfaces.srv import Resume
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Trigger
    from .save_map import SaveMapClient

    bag = args.bag.expanduser().resolve()
    report = inspect_bag(bag, args.odom_source, args.ekf_warmup)
    share = Path(get_package_share_directory("slam_toolbox"))
    if not (share / "hardware_build.json").is_file():
        raise RuntimeError("Build and source the pinned SLAM backend before offline mapping")
    live_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    if args.domain_id == live_domain or not 0 <= args.domain_id <= 101:
        raise ValueError("Choose a separate ROS_DOMAIN_ID in 0..101 for offline replay")
    lease_path = Path(f"/tmp/duojin02-offline-domain-{args.domain_id}.lock")
    lease = lease_path.open("a")
    fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1")
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1")
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    log = output / f".offline-session-{os.getpid()}.log"
    launch = player = None
    rclpy.init(args=[])
    node = Node("offline_session_controller")
    status_client = node.create_client(Trigger, "/slam_toolbox/processing_status")
    relay_client = node.create_client(GetParameters, "/replay_tf_filter/get_parameters")
    ekf_client = node.create_client(GetParameters, "/ekf_filter_node/get_parameters") if args.odom_source == "ekf" else None
    resume_client = node.create_client(Resume, "/rosbag2_player/resume")
    playback_clock = None
    playback_status = None

    def clock_callback(message):
        nonlocal playback_clock
        playback_clock = message.clock.sec * 1000000000 + message.clock.nanosec

    node.create_subscription(Clock, "/clock", clock_callback, qos_profile_sensor_data)

    def status(deadline):
        future = status_client.call_async(Trigger.Request())
        response = await_response(node, future, deadline, "SLAM queue status", (launch,))
        return json.loads(response.message)

    try:
        with log.open("w") as stream:
            command = ["ros2", "launch", "duojin02_bringup", "offline_mapping.launch.py",
                       f"odom_source:={args.odom_source}", f"scan_start_time_ns:={report['scan_start_time_ns']}"]
            if args.slam_params_file:
                command.append(f"slam_params_file:={args.slam_params_file.resolve()}")
            launch = subprocess.Popen(command, env=environment, stdout=stream,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + args.startup_timeout
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError(f"Offline nodes stopped during startup; see {log}")
                if (status_client.wait_for_service(timeout_sec=min(0.2, remaining(deadline, "Offline startup"))) and
                    relay_client.wait_for_service(timeout_sec=min(0.2, remaining(deadline, "Offline startup"))) and
                    (ekf_client is None or ekf_client.wait_for_service(timeout_sec=min(0.2, remaining(deadline, "Offline startup"))))):
                    break
            else:
                raise TimeoutError(f"Offline nodes did not become ready; see {log}")
            topics = ["/scan", "/tf", "/tf_static"]
            topics += ["/odom", "/imu"] if args.odom_source == "ekf" else ["/odometry/filtered"]
            player = subprocess.Popen(
                ["ros2", "bag", "play", str(bag), "--clock", "100", "--rate", str(args.rate),
                 "--start-paused", "--disable-keyboard-controls", "--wait-for-all-acked", "2000",
                 "--topics", *topics, "--remap", "/tf:=/replay/tf_in", "/tf_static:=/replay/tf_static_in",
                 "/scan:=/replay/scan_in"],
                env=environment, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            # Activate recorded time before TF arrives: its first backward jump
            # clears TF buffers, including transforms that would otherwise arrive early.
            while time.monotonic() < deadline:
                if player.poll() is not None or launch.poll() is not None:
                    raise RuntimeError(f"Offline nodes stopped while initializing recorded time; see {log}")
                rclpy.spin_once(node, timeout_sec=min(0.1, remaining(deadline, "Offline startup")))
                if playback_clock is not None and resume_client.wait_for_service(timeout_sec=min(0.1, remaining(deadline, "Offline startup"))):
                    current = status(deadline)
                    if current["clock_active"] and current["clock_nanoseconds"] == playback_clock:
                        break
            else:
                raise TimeoutError(f"SLAM did not activate the recorded clock; see {log}")
            future = resume_client.call_async(Resume.Request())
            await_response(node, future, deadline, "Bag player resume", (launch, player))
            playback_deadline = time.monotonic() + args.playback_timeout
            while player.poll() is None:
                if launch.poll() is not None:
                    raise RuntimeError(f"Offline nodes stopped during playback; see {log}")
                if time.monotonic() >= playback_deadline:
                    raise TimeoutError(f"Bag playback exceeded its timeout; see {log}")
                if playback_status is None:
                    playback_status = status_client.call_async(Trigger.Request())
                rclpy.spin_once(node, timeout_sec=min(0.1, remaining(playback_deadline, "Bag playback")))
                if playback_status.done():
                    if playback_status.exception():
                        raise playback_status.exception()
                    current = json.loads(playback_status.result().message)
                    playback_status = None
                    if current["invalid"]:
                        raise RuntimeError(f"SLAM rejected invalid inputs: {current}; see {log}")
            if playback_status is not None:
                playback_status.cancel()
                playback_status = None
            if player.returncode:
                raise RuntimeError(f"Bag player exited with {player.returncode}; see {log}")
            deadline = time.monotonic() + args.drain_timeout
            while time.monotonic() < deadline:
                current = status(deadline)
                if current["invalid"]:
                    raise RuntimeError(f"Invalid SLAM input: {current}; see {log}")
                if (current["received"] == report["scan_count"] and
                    current["queued"] == 0 and not current["processing"] and
                    current["accepted"] == current["completed"] and current["completed"] >= 2):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                raise TimeoutError(f"SLAM did not drain all recorded scans: {current}; see {log}")
            save = SaveMapClient(parameter_overrides=[
                Parameter("output_dir", value=str(output)),
                Parameter("map_name", value=args.map_name),
                Parameter("save_timeout", value=args.save_timeout),
            ])
            try:
                save_result = save.run()
            finally:
                save.destroy_node()
            if save_result:
                raise RuntimeError("Snapshot export failed; inspect the save_map_client_node error above")
            report.update(queue=current, rate=args.rate, domain_id=args.domain_id, clock_primed=True,
                          backend=json.loads((share / "hardware_build.json").read_text()),
                          log=str(log))
            report_file = output / f"offline-session-{os.getpid()}.json"
            report_file.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
            print(json.dumps(report, indent=2))
            return 0
    finally:
        if playback_status is not None:
            playback_status.cancel()
        stop_process(player)
        stop_process(launch, launch=True)
        node.destroy_node()
        rclpy.shutdown()
        lease.close()


def main():
    from .save_map import DEFAULT_SAVE_TIMEOUT

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--odom-source", choices=("recorded", "ekf"), default="recorded")
    parser.add_argument("--output-dir", type=Path, default=Path(os.environ.get("DUOJIN02_WORKSPACE_ROOT", ".")) / "maps")
    parser.add_argument("--map-name", default="auto")
    parser.add_argument("--slam-params-file", type=Path)
    parser.add_argument("--domain-id", type=int, default=97)
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument("--ekf-warmup", type=float, default=0.5,
                        help="Skip this many seconds of scans while replayed odom/IMU initialize EKF; recorded route skips none")
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--drain-timeout", type=float, default=60.0)
    parser.add_argument("--playback-timeout", type=float, default=3600.0)
    parser.add_argument("--save-timeout", type=float, default=DEFAULT_SAVE_TIMEOUT)
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be finite and positive")
    if not math.isfinite(args.ekf_warmup) or args.ekf_warmup < 0:
        parser.error("--ekf-warmup must be finite and nonnegative")
    for field in ("startup_timeout", "drain_timeout", "playback_timeout", "save_timeout"):
        if not math.isfinite(getattr(args, field)) or getattr(args, field) <= 0:
            parser.error(f"--{field.replace('_', '-')} must be finite and positive")
    try:
        return run(args)
    except (Exception, KeyboardInterrupt) as error:
        print(f"Offline mapping failed: {error}", flush=True)
        return 1
