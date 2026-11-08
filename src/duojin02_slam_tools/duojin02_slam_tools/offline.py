"""Reconstruct a map from a bag in an isolated ROS domain and wait for queue drain."""

import argparse
import fcntl
import json
import math
import os
import re
import signal
import subprocess
import time
import uuid
from pathlib import Path

import yaml

from .playback_flow import PlaybackFlow


class GeometryDropAudit:
    """Incrementally audit explicit pre-dispatch rejections without rereading a growing log.

    The native queue counters start after physical geometry preparation. Its rejected
    source IDs therefore supplement `received`; they are not invalid native scans,
    queued work, completed work, or keyframe-selection omissions.
    """

    def __init__(self, path, expected_stamps):
        self.path = Path(path)
        self.expected_stamps = set(expected_stamps)
        self.offset = 0
        self.pending = ""
        self.drops = {}
        self.duplicate_lines = 0

    def update(self):
        if not self.path.exists():
            return
        with self.path.open(errors="replace") as stream:
            stream.seek(self.offset)
            text = self.pending + stream.read()
            self.offset = stream.tell()
        lines = text.split("\n")
        self.pending = lines.pop()
        for line in lines:
            if "scan_geometry_drop" not in line:
                continue
            stamp = re.search(r"raw_stamp=(\d+)", line)
            reason = re.search(r"reason=(.*?)(?: waited_sec=| raw_stamp=|$)", line)
            if stamp is None or reason is None:
                raise RuntimeError(f"Incomplete geometry rejection record: {line}")
            stamp = int(stamp.group(1))
            if stamp not in self.expected_stamps:
                raise RuntimeError(f"Geometry rejection refers to a scan outside this bag: {stamp}")
            value = reason.group(1).strip()
            if value == "pending_queue_limit":
                raise RuntimeError("Physical scan queue overflowed; replay is incomplete")
            if stamp in self.drops:
                self.duplicate_lines += 1
                if self.drops[stamp] != value:
                    raise RuntimeError(f"Conflicting geometry rejections for source scan {stamp}")
            self.drops[stamp] = value

    def accounting(self, current, expected_count):
        self.update()
        accounted = int(current["received"]) + len(self.drops)
        if accounted > expected_count:
            raise RuntimeError(f"SLAM source accounting exceeds bag scan count: {accounted}>{expected_count}")
        return {
            "input_count": expected_count,
            "post_geometry_received_count": current["received"],
            "geometry_rejected_count": len(self.drops),
            "geometry_rejected_raw_stamps": sorted(self.drops),
            "geometry_rejection_reasons": {str(key): self.drops[key] for key in sorted(self.drops)},
            "duplicate_rejection_log_lines": self.duplicate_lines,
            "accounted_input_count": accounted,
            "input_accounting_closed": accounted == expected_count,
        }


def queue_drained(current, accounting):
    return (
        accounting["input_accounting_closed"]
        and current["invalid"] == 0
        and current["queued"] == 0
        and not current["processing"]
        and current["accepted"] == current["completed"]
        and current["completed"] >= 2
        and current.get("physical_pending", 0) == 0
    )


def remaining(deadline, description):
    value = deadline - time.monotonic()
    if value <= 0:
        raise TimeoutError(f"{description} exceeded its timeout")
    return value


class OfflineProcessStopped(RuntimeError):
    """A recorded child exit, distinct from invalid native queue evidence."""


def await_response(node, future, deadline, description, processes=()):
    import rclpy

    try:
        while not future.done():
            if any(process.poll() is not None for process in processes):
                raise OfflineProcessStopped(f"Offline process stopped while waiting for {description}")
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
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {
        "/scan": "sensor_msgs/msg/LaserScan",
        "/tf_static": "tf2_msgs/msg/TFMessage",
    }
    required.update(
        {"/tf": "tf2_msgs/msg/TFMessage"}
        if source == "recorded"
        else {"/odom": "nav_msgs/msg/Odometry", "/imu": "sensor_msgs/msg/Imu"}
    )
    for name, message_type in required.items():
        if topics.get(name) != message_type:
            raise ValueError(f"Bag requires {name} ({message_type}) for odom_source={source}")
    count = sum(
        entry["message_count"]
        for entry in metadata["topics_with_message_count"]
        if entry["topic_metadata"]["name"] == "/scan"
    )
    if count < 2:
        raise ValueError("Bag must contain at least two laser scans")

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
                links.add(
                    (
                        transform.header.frame_id.lstrip("/"),
                        transform.child_frame_id.lstrip("/"),
                    )
                )
    reachable = {"base_footprint"}
    for _ in range(len(links) + 1):
        reachable.update(child for parent, child in links if parent in reachable)
    if not sensor_frames <= reachable:
        raise ValueError(
            f"Bag has no base_footprint TF chain to sensors: {sorted(sensor_frames - reachable)}"
        )
    if source == "recorded" and ("odom", "base_footprint") not in links:
        raise ValueError("Recorded odometry requires odom -> base_footprint TF")
    if len(scan_stamps) != count or any(
        later <= earlier for earlier, later in zip(scan_stamps, scan_stamps[1:])
    ):
        raise ValueError("Bag scan timestamps must be strictly increasing and match its metadata")
    start = scan_stamps[0] + int(warmup * 1000000000) if source == "ekf" else 0
    skipped = sum(stamp < start for stamp in scan_stamps)
    if count - skipped < 2:
        raise ValueError("Bag has fewer than two scans after EKF warmup")
    return {
        "bag": str(path),
        "storage_id": storage,
        "scan_count": count - skipped,
        "recorded_scan_count": count,
        "warmup_skipped_scans": skipped,
        "scan_start_time_ns": start,
        "odom_source": source,
        "topics": topics,
        "source_scan_stamps_ns": [stamp for stamp in scan_stamps if stamp >= start],
    }


_OWNER_ENV = "DUOJIN02_SLAM_OFFLINE_PROCESS_OWNER"


def _process_identity(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(pid), int(fields[19]), int(fields[3]), int(fields[2]), fields[0])
    except (FileNotFoundError, ProcessLookupError):
        return None


def _process_token(pid):
    try:
        prefix = (_OWNER_ENV + "=").encode()
        for value in Path(f"/proc/{pid}/environ").read_bytes().split(b"\0"):
            if value.startswith(prefix):
                return value[len(prefix):].decode("ascii")
    except (FileNotFoundError, ProcessLookupError):
        return None
    return None


def spawn_owned_process(command, **kwargs):
    """Own a fresh session and descendants retaining this private inherited tag.

    The tag is an accidental PID-reuse guard, not a security boundary against
    another process of the same user. A child that clears it and escapes the
    original session is outside this deliberately small helper's guarantee.
    """
    if not hasattr(os, "pidfd_open") or not hasattr(signal, "pidfd_send_signal"):
        raise RuntimeError("Offline process ownership requires Linux pidfd support")
    if kwargs.pop("start_new_session", True) is not True:
        raise ValueError("Owned offline processes require a new session")
    supplied_environment = kwargs.pop("env", None)
    environment = dict(os.environ if supplied_environment is None else supplied_environment)
    token = uuid.uuid4().hex
    environment[_OWNER_ENV] = token
    process = subprocess.Popen(command, env=environment, start_new_session=True, **kwargs)
    identity = _process_identity(process.pid)
    process._slam_process_owner = {"token": token, "sid": process.pid, "pgid": process.pid,
                                   "leader_start_ticks": None if identity is None else identity[1]}
    return process


def _owned_members(owner):
    members = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        identity = _process_identity(int(entry.name))
        if identity is None or identity[4] == "Z":
            continue
        try:
            token = _process_token(identity[0])
        except PermissionError:
            if identity[2] == owner["sid"]:
                # Linux can revoke environ access before an exiting task becomes
                # a zombie. Allow that short transition, but still refuse a live
                # process whose ownership cannot be verified.
                deadline = time.monotonic() + 0.05
                while True:
                    current = _process_identity(identity[0])
                    if current is None or current[4] == "Z":
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("Cannot verify member of the owned offline session")
                    time.sleep(0.005)
            continue
        if token == owner["token"]:
            if (identity[0] == owner["sid"] and owner["leader_start_ticks"] is not None
                    and identity[1] != owner["leader_start_ticks"]):
                raise RuntimeError("Owned offline leader identity changed")
            members.append(identity)
        elif identity[2] == owner["sid"]:
            # A process can release its environment while exiting before /proc
            # reports Z. Count it as still present, but never signal it unless
            # its tag is independently verified below.
            members.append(identity)
    return members


def _signal_owned(owner, sig, leader_only=False):
    for identity in _owned_members(owner):
        if leader_only and identity[0] != owner["sid"]:
            continue
        try:
            descriptor = os.pidfd_open(identity[0])
        except ProcessLookupError:
            continue
        try:
            # Opening a pidfd and then rechecking start ticks closes the PID reuse
            # gap between /proc enumeration and signal delivery.
            current = _process_identity(identity[0])
            if current is not None and current[:4] == identity[:4] and current[4] != "Z":
                token = _process_token(identity[0])
                if token == owner["token"]:
                    signal.pidfd_send_signal(descriptor, sig)
                elif token is not None:
                    raise RuntimeError("Unverified process remains in the owned offline session")
        except ProcessLookupError:
            pass
        finally:
            os.close(descriptor)


def _wait_owned_empty(process, owner, timeout):
    deadline = time.monotonic() + timeout
    while True:
        process.poll()  # Reap the leader even when its children outlive it.
        if not _owned_members(owner):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.02)


def stop_process(process, launch=False):
    if process is None:
        return
    owner = getattr(process, "_slam_process_owner", None)
    if owner is not None:
        leader_running = process.poll() is None
        if leader_running:
            # ros2 launch forwards this one INT to its own nodes. Sending INT to
            # both the wrapper and its children would deliver duplicate signals.
            _signal_owned(owner, signal.SIGINT, leader_only=launch)
            if _wait_owned_empty(process, owner, 10):
                return
        elif not _owned_members(owner):
            return
        _signal_owned(owner, signal.SIGTERM)
        if _wait_owned_empty(process, owner, 5):
            return
        _signal_owned(owner, signal.SIGKILL)
        if not _wait_owned_empty(process, owner, 5):
            raise RuntimeError("Owned offline processes survived bounded cleanup")
        return
    # Compatibility for existing callers that supply an untagged Popen. Only
    # spawn_owned_process can prove ownership after its leader has exited.
    if process.poll() is not None:
        return
    try:

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

        return


def run(args):
    from ament_index_python.packages import get_package_share_directory
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from rosbag2_interfaces.srv import IsPaused, Pause, Resume
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
    drop_audit = GeometryDropAudit(log, report.pop("source_scan_stamps_ns"))
    launch = player = None
    rclpy.init(args=[])
    node = Node("offline_session_controller")
    status_client = node.create_client(Trigger, "/slam_toolbox/processing_status")
    relay_client = node.create_client(GetParameters, "/replay_tf_filter/get_parameters")
    ekf_client = (
        node.create_client(GetParameters, "/ekf_filter_node/get_parameters")
        if args.odom_source == "ekf"
        else None
    )
    resume_client = node.create_client(Resume, "/rosbag2_player/resume")
    pause_client = node.create_client(Pause, "/rosbag2_player/pause")
    paused_client = node.create_client(IsPaused, "/rosbag2_player/is_paused")
    playback_clock = None
    playback_status = None
    playback_status_started = None
    flow = PlaybackFlow()
    flow_transport = []
    flow_status_peak_sec = 0.0
    actual_playback_wall_s = actual_drain_wall_s = None
    outcome = "incomplete"
    failure = None
    flow_path = output / f"offline-flow-{os.getpid()}.json"

    def save_flow():
        value = flow.report(time.monotonic())
        value.update(outcome=outcome, failure=failure, requested_max_rate=args.rate,
                     status_timeout_sec=args.flow_control_timeout,
                     peak_status_response_sec=flow_status_peak_sec, transport=flow_transport,
                     actual_playback_wall_s=actual_playback_wall_s, actual_drain_wall_s=actual_drain_wall_s)
        flow_path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")

    def clock_callback(message):
        nonlocal playback_clock
        playback_clock = message.clock.sec * 1000000000 + message.clock.nanosec

    node.create_subscription(Clock, "/clock", clock_callback, qos_profile_sensor_data)

    def status(deadline):
        future = status_client.call_async(Trigger.Request())
        response = await_response(node, future, deadline, "SLAM queue status", (launch,))
        return json.loads(response.message)

    def control_source(action, deadline):
        """Confirm a producer command, then observe its actual dependency boundary."""
        record = {"action": action, "requested_wall_monotonic_s": time.monotonic(),
                  "acknowledged": False}
        flow_transport.append(record)
        save_flow()
        local_deadline = min(deadline, time.monotonic() + args.flow_control_timeout)
        try:
            client, request = (pause_client, Pause.Request()) if action.startswith("pause_") else (resume_client, Resume.Request())
            await_response(node, client.call_async(request), local_deadline, "Bag producer " + action, (launch, player))
            observed = await_response(node, paused_client.call_async(IsPaused.Request()), local_deadline,
                                      "Bag producer state confirmation", (launch, player))
            expected = action.startswith("pause_")
            if bool(observed.paused) != expected:
                raise RuntimeError("Bag producer pause acknowledgement disagrees with its state")
            current = status(local_deadline)
            record.update(acknowledged=True, paused=expected, confirmed_wall_monotonic_s=time.monotonic(),
                          clock_nanoseconds=current["clock_nanoseconds"], physical_pending=current["physical_pending"],
                          outstanding=current["outstanding"])
            next_action = flow.acknowledged(action, current, time.monotonic())
            save_flow()
            return next_action
        except Exception as error:
            record["error"] = str(error)
            save_flow()
            # EOF can race a pause request after all messages have been sent.
            # Native input accounting, checked below, still decides completion.
            if isinstance(error, OfflineProcessStopped) and player.poll() == 0:
                record["completed_during_command"] = True
                return None
            raise

    try:
        with log.open("w") as stream:
            command = [
                "ros2",
                "launch",
                "duojin02_bringup",
                "offline_mapping.launch.py",
                f"odom_source:={args.odom_source}",
                f"scan_start_time_ns:={report['scan_start_time_ns']}",
            ]
            if args.slam_params_file:
                command.append(f"slam_params_file:={args.slam_params_file.resolve()}")
            launch = spawn_owned_process(
                command,
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + args.startup_timeout
            while time.monotonic() < deadline:
                if launch.poll() is not None:
                    raise RuntimeError(f"Offline nodes stopped during startup; see {log}")
                if (
                    status_client.wait_for_service(
                        timeout_sec=min(0.2, remaining(deadline, "Offline startup"))
                    )
                    and relay_client.wait_for_service(
                        timeout_sec=min(0.2, remaining(deadline, "Offline startup"))
                    )
                    and (
                        ekf_client is None
                        or ekf_client.wait_for_service(
                            timeout_sec=min(0.2, remaining(deadline, "Offline startup"))
                        )
                    )
                ):
                    break
            else:
                raise TimeoutError(f"Offline nodes did not become ready; see {log}")
            topics = ["/scan", "/tf", "/tf_static"]
            topics += ["/odom", "/imu"] if args.odom_source == "ekf" else ["/odometry/filtered"]
            player = spawn_owned_process(
                [
                    "ros2",
                    "bag",
                    "play",
                    str(bag),
                    "--clock",
                    "100",
                    "--rate",
                    str(args.rate),
                    "--start-paused",
                    "--disable-keyboard-controls",
                    "--wait-for-all-acked",
                    "2000",
                    "--topics",
                    *topics,
                    "--remap",
                    "/tf:=/replay/tf_in",
                    "/tf_static:=/replay/tf_static_in",
                    "/scan:=/replay/scan_in",
                ],
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

            while time.monotonic() < deadline:
                if player.poll() is not None or launch.poll() is not None:
                    raise RuntimeError(
                        f"Offline nodes stopped while initializing recorded time; see {log}"
                    )
                rclpy.spin_once(node, timeout_sec=min(0.1, remaining(deadline, "Offline startup")))
                if playback_clock is not None and resume_client.wait_for_service(
                    timeout_sec=min(0.1, remaining(deadline, "Offline startup"))
                ) and pause_client.wait_for_service(
                    timeout_sec=min(0.1, remaining(deadline, "Offline startup"))
                ) and paused_client.wait_for_service(
                    timeout_sec=min(0.1, remaining(deadline, "Offline startup"))
                ):
                    current = status(deadline)
                    if current["clock_active"] and current["clock_nanoseconds"] == playback_clock:
                        flow.inspect(current)
                        break
            else:
                raise TimeoutError(f"SLAM did not activate the recorded clock; see {log}")
            playback_started_wall_s = time.monotonic()
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
                    playback_status_started = time.monotonic()
                rclpy.spin_once(
                    node,
                    timeout_sec=min(0.1, remaining(playback_deadline, "Bag playback")),
                )
                if playback_status.done():
                    if playback_status.exception():
                        raise playback_status.exception()
                    current = json.loads(playback_status.result().message)
                    flow_status_peak_sec = max(flow_status_peak_sec, time.monotonic() - playback_status_started)
                    playback_status = None
                    if current["invalid"]:
                        raise RuntimeError(f"SLAM rejected invalid inputs: {current}; see {log}")
                    action = flow.request(current)
                    # A newly arrived physical dependency after pause ACK takes
                    # priority over holding the low-watermark drain interval.
                    while action is not None and player.poll() is None:
                        action = control_source(action, playback_deadline)
                elif time.monotonic() - playback_status_started >= args.flow_control_timeout:
                    # Stop supplying new work before failing a stale consumer.
                    # This is a failed run, never a successful drained boundary.
                    try:
                        local = min(playback_deadline, time.monotonic() + args.flow_control_timeout)
                        await_response(node, pause_client.call_async(Pause.Request()), local,
                                       "Emergency producer pause after stale queue status", (launch, player))
                    finally:
                        raise TimeoutError(f"SLAM queue status exceeded {args.flow_control_timeout}s; see {log}")
            if playback_status is not None:
                playback_status.cancel()
                playback_status = None
            if player.returncode:
                raise RuntimeError(f"Bag player exited with {player.returncode}; see {log}")
            producer_eof_wall_s = time.monotonic()
            actual_playback_wall_s = producer_eof_wall_s - playback_started_wall_s
            flow.finished(producer_eof_wall_s)
            deadline = time.monotonic() + args.drain_timeout
            while time.monotonic() < deadline:
                current = status(deadline)
                flow.inspect(current)
                if current["invalid"]:
                    raise RuntimeError(f"Invalid SLAM input: {current}; see {log}")
                accounting = drop_audit.accounting(current, report["scan_count"])
                if queue_drained(current, accounting):
                    break
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                raise TimeoutError(f"SLAM did not drain all recorded scans: {current}; see {log}")
            actual_drain_wall_s = time.monotonic() - producer_eof_wall_s
            save = SaveMapClient(
                parameter_overrides=[
                    Parameter("output_dir", value=str(output)),
                    Parameter("map_name", value=args.map_name),
                    Parameter("save_timeout", value=args.save_timeout),
                ]
            )
            try:
                save_result = save.run()
            finally:
                save.destroy_node()
            if save_result:
                raise RuntimeError(
                    "Snapshot export failed; inspect the save_map_client_node error above"
                )
            report.update(
                queue=current,
                geometry_rejected_raw_stamps=accounting["geometry_rejected_raw_stamps"],
                input_accounting=accounting,
                accounting_contract="native post-geometry received + unique explicit geometry-rejected bag source IDs == input count; zero queued/in-flight work and accepted == completed; keyframe omissions are not rejections",
                rate=args.rate,
                rate_semantics="maximum source rate; bounded producer pauses are included in wall time",
                producer_flow=flow.report(time.monotonic()),
                producer_flow_path=str(flow_path),
                actual_playback_wall_s=actual_playback_wall_s,
                actual_drain_wall_s=actual_drain_wall_s,
                phase_wall_semantics="controller monotonic intervals: initial Resume request to observed successful producer exit; exit to complete native/geometry input closure",
                domain_id=args.domain_id,
                clock_primed=True,
                backend=json.loads((share / "hardware_build.json").read_text()),
                log=str(log),
            )
            report_file = output / f"offline-session-{os.getpid()}.json"
            report_file.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
            print(json.dumps(report, indent=2))
            outcome = "completed"
            return 0
    except BaseException as error:
        failure = f"{type(error).__name__}: {error}"
        outcome = "failed"
        raise
    finally:
        cleanup_errors = []
        actions = [
            lambda: playback_status.cancel() if playback_status is not None else None,
            lambda: stop_process(player),
            lambda: stop_process(launch, launch=True),
            node.destroy_node, rclpy.shutdown, lease.close,
        ]
        for action in actions:
            try:
                action()
            except BaseException as error:
                cleanup_errors.append(error)
        if cleanup_errors:
            prior = "" if failure is None else failure + "; "
            failure = prior + "cleanup_failed: " + "; ".join(
                f"{type(error).__name__}: {error}" for error in cleanup_errors)
            outcome = "failed"
        try:
            save_flow()
        except BaseException as error:
            cleanup_errors.append(error)
        if cleanup_errors:
            raise RuntimeError(f"Offline cleanup failed: {failure or cleanup_errors[0]}") from cleanup_errors[0]


def main():
    from .save_map import DEFAULT_SAVE_TIMEOUT

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--odom-source", choices=("recorded", "ekf"), default="recorded")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(os.environ.get("DUOJIN02_WORKSPACE_ROOT", ".")) / "maps",
    )
    parser.add_argument("--map-name", default="auto")
    parser.add_argument("--slam-params-file", type=Path)
    parser.add_argument("--domain-id", type=int, default=97)
    parser.add_argument("--rate", type=float, default=0.5,
                        help="Maximum source playback multiplier; dependency-aware pauses add wall time")
    parser.add_argument(
        "--ekf-warmup",
        type=float,
        default=0.5,
        help="Skip this many seconds of scans while replayed odom/IMU initialize EKF; recorded route skips none",
    )
    parser.add_argument("--startup-timeout", type=float, default=60.0)
    parser.add_argument("--drain-timeout", type=float, default=60.0)
    parser.add_argument("--playback-timeout", type=float, default=3600.0)
    parser.add_argument("--save-timeout", type=float, default=DEFAULT_SAVE_TIMEOUT)
    parser.add_argument("--flow-control-timeout", type=float, default=2.0,
                        help="Finite producer command/consumer status deadline; pauses preserve original data and clock")
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be finite and positive")
    if not math.isfinite(args.ekf_warmup) or args.ekf_warmup < 0:
        parser.error("--ekf-warmup must be finite and nonnegative")
    for field in (
        "startup_timeout",
        "drain_timeout",
        "playback_timeout",
        "save_timeout",
        "flow_control_timeout",
    ):
        if not math.isfinite(getattr(args, field)) or getattr(args, field) <= 0:
            parser.error(f"--{field.replace('_', '-')} must be finite and positive")
    try:
        return run(args)
    except (Exception, KeyboardInterrupt) as error:
        print(f"Offline mapping failed: {error}", flush=True)
        return 1
