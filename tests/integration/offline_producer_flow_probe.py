"""Real rosbag2 Pause/Resume/clock/order control with a deliberately slow FIFO.

This is a transport fixture, not a localization estimator or LiLoc score.
"""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from collections import deque

import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosbag2_interfaces.srv import IsPaused, Pause, Resume
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from duojin02_slam_tools.offline import OfflineProcessStopped, await_response, spawn_owned_process, stop_process
from duojin02_slam_tools.playback_flow import PlaybackFlow
from ros_isolation import configure_isolation


def main():
    out = Path(sys.argv[1]);out.mkdir(parents=True, exist_ok=False)
    dependency_reopen = sys.argv[2:] == ["--dependency-reopen"]
    if sys.argv[2:] and not dependency_reopen:
        raise ValueError("Unknown transport fixture option")
    production_paths = [Path(sys.modules[name].__file__).resolve() for name in
                        ("duojin02_slam_tools.offline", "duojin02_slam_tools.playback_flow")]
    production_before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in production_paths}
    configure_isolation()
    bag = out / "source"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    writer.create_topic(rosbag2_py.TopicMetadata(name="/flow_probe/scan", type="sensor_msgs/msg/LaserScan", serialization_format="cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(name="/flow_probe/future_tf", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"))
    expected = []
    for index in range(80):
        ns = 10_000_000_000 + index * 50_000_000
        scan = LaserScan()
        scan.header.frame_id = "transport_fixture"
        scan.header.stamp.sec = ns // 1_000_000_000
        scan.header.stamp.nanosec = ns % 1_000_000_000
        scan.angle_min = 0.0;scan.angle_max = 0.0;scan.angle_increment = 0.1
        scan.range_min = 0.1;scan.range_max = 10.0;scan.ranges = [1.0 + index / 100.0]
        data = serialize_message(scan)
        expected.append(hashlib.sha256(data).hexdigest())
        writer.write("/flow_probe/scan", data, ns)
        if index == 16:
            transform = TransformStamped()
            transform.header.stamp = scan.header.stamp
            transform.header.frame_id = "fixture_odom"
            transform.child_frame_id = "fixture_base"
            transform.transform.rotation.w = 1.0
            marker = TFMessage();marker.transforms = [transform]
            writer.write("/flow_probe/future_tf", serialize_message(marker), ns)
    del writer
    rclpy.init()
    node = Node("offline_producer_transport_fixture")
    flow = PlaybackFlow();clock = [None];received = [];pending = deque()
    state = dict(accepted=0, completed=0, invalid=0, processing=False)
    dependency = dict(armed=False, future_received=False, pending=0)
    mutex = threading.Lock();stop = threading.Event();capacity = 16

    def consume(data):
        received.append(hashlib.sha256(data).hexdigest())
        with mutex:
            if len(pending) >= capacity:
                state["invalid"] += 1
            else:
                pending.append(data);state["accepted"] += 1

    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(LaserScan, "/flow_probe/scan", consume, qos, raw=True)
    def future_tf(message):
        dependency["future_received"] = True
        dependency["pending"] = 0
    node.create_subscription(TFMessage, "/flow_probe/future_tf", future_tf, qos)
    clock_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Clock, "/clock", lambda msg: clock.__setitem__(0, msg.clock.sec * 1_000_000_000 + msg.clock.nanosec), clock_qos)

    def snapshot():
        with mutex:
            return dict(schema=PlaybackFlow.SCHEMA, capacity=capacity, physical_pending=dependency["pending"],
                        received=len(received), queued=len(pending), outstanding=state["accepted"]-state["completed"],
                        clock_nanoseconds=clock[0] or 0, clock_active=clock[0] is not None, **state)

    def service(request, response):
        response.success = True;response.message = json.dumps(snapshot());return response

    node.create_service(Trigger, "/flow_probe/status", service)
    status_client = node.create_client(Trigger, "/flow_probe/status")
    pause_client = node.create_client(Pause, "/rosbag2_player/pause")
    resume_client = node.create_client(Resume, "/rosbag2_player/resume")
    paused_client = node.create_client(IsPaused, "/rosbag2_player/is_paused")

    def worker():
        while not stop.is_set():
            item = None
            with mutex:
                if pending:
                    item = pending.popleft();state["processing"] = True
            if item is None:
                stop.wait(.002);continue
            # 14.3 messages/s is below the 40/s maximum producer rate.
            if stop.wait(.07):
                return
            with mutex:
                state["processing"] = False;state["completed"] += 1

    thread = threading.Thread(target=worker);thread.start()
    player = None;stable_paused_clock_samples = 0;last_paused_clock = None;last_pause_receipt = 0
    started = time.monotonic();ledger = []
    try:
        with (out / "player.log").open("x") as log:
            player = spawn_owned_process(["ros2", "bag", "play", str(bag), "--clock", "100", "--rate", "2",
                                       "--start-paused", "--disable-keyboard-controls", "--wait-for-all-acked", "2000"],
                                      stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            pid_record = dict(pid=player.pid, start_ticks=Path(f"/proc/{player.pid}/stat").read_text().rsplit(")", 1)[1].split()[19])
            (out / "process-owner.json").write_text(json.dumps(pid_record, indent=2) + "\n")
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if clock[0] is not None and all(c.service_is_ready() for c in (pause_client, resume_client, paused_client, status_client)):
                    break
                assert player.poll() is None, "Player stopped at startup"
            else:
                raise TimeoutError("Producer services or recorded clock unavailable")

            def call(client, request):
                return await_response(node, client.call_async(request), time.monotonic() + 2, "transport fixture RPC", (player,))

            def read_status():
                return json.loads(call(status_client, Trigger.Request()).message)

            flow.inspect(read_status())
            assert call(paused_client, IsPaused.Request()).paused
            call(resume_client, Resume.Request())
            while player.poll() is None:
                assert time.monotonic() - started < 45, "Transport fixture budget exceeded"
                try:
                    current = read_status()
                except OfflineProcessStopped:
                    if player.poll() == 0:
                        break
                    raise
                action = flow.request(current)
                if flow.paused and time.monotonic() - last_pause_receipt > .05:
                    if last_paused_clock == clock[0]:
                        stable_paused_clock_samples += 1
                    last_paused_clock = clock[0]
                while action is not None and player.poll() is None:
                    pause = action.startswith("pause_")
                    call(pause_client if pause else resume_client, Pause.Request() if pause else Resume.Request())
                    assert call(paused_client, IsPaused.Request()).paused == pause
                    if pause and dependency_reopen and not dependency["armed"]:
                        assert not dependency["future_received"], "Fixture did not pause before its future TF marker"
                        dependency["armed"] = True
                        dependency["pending"] = 1
                    current = read_status()
                    ledger.append(dict(action=action, clock_ns=clock[0], wall_s=time.monotonic(), status=current))
                    if pause:
                        last_pause_receipt = time.monotonic();last_paused_clock = None
                    action = flow.acknowledged(action, current, time.monotonic())
                rclpy.spin_once(node, timeout_sec=.005)
            assert player.returncode == 0
            flow.finished(time.monotonic())
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
                current = snapshot()
                flow.inspect(current)
                if len(received) == len(expected) and current["completed"] == len(expected):
                    break
            assert received == expected, "Source CDR membership/order changed"
            assert state["invalid"] == 0 and snapshot()["outstanding"] == 0
            report = flow.report(time.monotonic())
            assert report["pause_count"] > 0 and report["peak_queued"] < capacity
            if dependency_reopen:
                assert dependency["armed"] and dependency["future_received"] and dependency["pending"] == 0
                assert report["dependency_reopens"] > 0, "Producer did not reopen for the actual future TF marker"
            assert stable_paused_clock_samples > 0, "No stable paused recorded-clock interval was observed"
            result = dict(passed=True, scope="Real rosbag2 producer RPCs/paused clock and exact CDR order with a synthetic slow FIFO; no estimator or LiLoc score",
                          source_messages=len(expected), received_messages=len(received), processed_messages=state["completed"],
                          source_cdr_sha256=expected, received_cdr_sha256=received, invalid=state["invalid"],
                          stable_paused_clock_samples=stable_paused_clock_samples, producer_flow=report,
                          dependency_reopen_fixture=dependency_reopen, dependency=dependency,
                          transport=ledger, elapsed_wall_s=time.monotonic()-started,
                          source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
            assert production_before == {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in production_paths}
            result["production_sources_sha256"] = production_before
            (out / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps({key:result[key] for key in ("passed", "source_messages", "processed_messages", "stable_paused_clock_samples", "elapsed_wall_s")}))
    except Exception as error:
        (out / "failure.json").write_text(json.dumps(dict(
            passed=False, error_type=type(error).__name__, error=str(error),
            status=snapshot(), producer_flow=flow.report(time.monotonic()),
            transport=ledger, elapsed_wall_s=time.monotonic()-started), indent=2) + "\n")
        raise
    finally:
        stop_process(player);stop.set();thread.join(timeout=2)
        node.destroy_node();rclpy.shutdown()


if __name__ == "__main__":
    main()
