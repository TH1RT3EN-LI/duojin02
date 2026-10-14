import os
import json
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rosbag2_py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def run():
    os.environ["ROS_DOMAIN_ID"] = "93"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()

    nodes = [
        Node(name, start_parameter_services=False)
        for name in ("base_driver", "lslidar_driver_node", "ekf_filter_node", "slam_toolbox")
    ]
    publisher = nodes[0]
    qos = QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
    )
    static = publisher.create_publisher(TFMessage, "/tf_static", qos)
    scan = publisher.create_publisher(LaserScan, "/scan", 100)
    transform = TransformStamped()
    transform.header.frame_id = "base_footprint"
    transform.child_frame_id = "laser"
    transform.transform.rotation.w = 1.0
    static.publish(TFMessage(transforms=[transform]))
    from rclpy.executors import MultiThreadedExecutor
    import threading

    executor = MultiThreadedExecutor(num_threads=2)
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        with tempfile.TemporaryDirectory() as folder:
            bag = Path(folder) / "captured"
            with (Path(folder) / "record.log").open("w") as log:
                process = subprocess.Popen(
                    ["./scripts/record_slam.sh", str(bag)],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    began = time.monotonic()
                    deadline = time.monotonic() + 25
                    while scan.get_subscription_count() == 0 and time.monotonic() < deadline:
                        time.sleep(0.1)
                    assert scan.get_subscription_count(), (Path(folder) / "record.log").read_text()
                    assert time.monotonic() - began < 8, "metadata delayed sensor recording"
                    for _ in range(12):
                        message = LaserScan()
                        message.header.frame_id = "laser"
                        message.header.stamp = publisher.get_clock().now().to_msg()
                        message.ranges = [1.0, 2.0]
                        scan.publish(message)
                        time.sleep(0.05)
                    time.sleep(0.5)
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                    returncode = process.wait(timeout=10)
                    assert returncode == 130, f"Recording did not finish after SIGINT: {returncode}"
                    print(
                        json.dumps(
                            {"recording_cleanup": {"returncode": returncode, "forced": False}}
                        )
                    )
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions("", ""),
            )
            topics = []
            while reader.has_next():
                topic, _, _ = reader.read_next()
                topics.append(topic)
            assert topics.count("/scan") >= 10, topics
            assert topics.count("/tf_static") >= 1, topics
            assert (Path(str(bag) + ".configuration") / "slam-build.json").is_file()
            configuration = Path(str(bag) + ".configuration")
            assert not list(
                configuration.glob("*.pending")
            ), "interrupted metadata left as a pending success"
            for name in ("base_driver", "lslidar_driver_node", "ekf_filter_node", "slam_toolbox"):
                assert (
                    configuration / f"{name}.error.log"
                ).read_text(), "missing metadata failure report"
            print(
                "late recorder captured transient static TF, live scans and backend provenance; "
                "unavailable metadata did not delay recording; interruption reports and cleanup: passed"
            )
    finally:
        stopped = executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        print(
            json.dumps(
                {
                    "recording_fixture_cleanup": {
                        "executor_stopped": stopped,
                        "worker_stopped": not thread.is_alive(),
                    }
                }
            )
        )
        assert stopped and not thread.is_alive(), "Recording fixture did not finish normally"


if __name__ == "__main__":
    run()
