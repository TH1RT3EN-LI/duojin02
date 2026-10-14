"""Exercise the compiled safety node with sensor faults; no physical devices."""

import json
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--guard-binary", type=Path)
    parser.add_argument(
        "--params-file",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "src/duojin02_bringup/config/safety.yaml",
    )
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = "92"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    node = Node("velocity_guard_probe")
    status = []
    output = []
    node.create_subscription(
        String, "/navigation/guard_status", lambda m: status.append(json.loads(m.data)), 10
    )
    node.create_subscription(Twist, "/cmd_vel_base", output.append, 10)
    cmd = node.create_publisher(Twist, "/cmd_vel_requested", 10)
    odom = node.create_publisher(Odometry, "/odometry/filtered", 10)
    scan = node.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    grid = node.create_publisher(
        OccupancyGrid,
        "/local_costmap/costmap",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    prior = node.create_publisher(
        OccupancyGrid, "/map", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    )
    covariance = node.create_publisher(
        PoseWithCovarianceStamped,
        "/amcl_pose",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    transforms = node.create_publisher(TFMessage, "/tf", 10)
    parameters = args.params_file
    with open("/tmp/velocity-guard-probe.log", "w") as log:
        binary = (
            args.guard_binary
            or Path(get_package_prefix("duojin02_navigation"))
            / "lib/duojin02_navigation/velocity_guard"
        )
        process = subprocess.Popen(
            [str(binary), "--ros-args", "--params-file", str(parameters)],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        def pump(
            duration,
            point=(1.5, 0),
            requested=0.3,
            actual=0.0,
            send_scan=True,
            send_command=True,
            send_odom=True,
            scan_min=0.02,
        ):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                stamp = node.get_clock().now().to_msg()
                tf = TransformStamped()
                tf.header.stamp = stamp
                tf.header.frame_id = "odom"
                tf.child_frame_id = "map"
                tf.transform.rotation.w = 1.0
                transforms.publish(TFMessage(transforms=[tf]))
                o = Odometry()
                o.header.stamp = stamp
                o.header.frame_id = "odom"
                o.child_frame_id = "base_footprint"
                o.pose.pose.orientation.w = 1.0
                o.twist.twist.linear.x = actual
                if send_odom:
                    odom.publish(o)
                if send_scan:
                    s = LaserScan()
                    s.header.stamp = stamp
                    s.header.frame_id = "odom"
                    s.range_min = scan_min
                    s.range_max = 8.0
                    s.angle_min = float(math.atan2(point[1], point[0])) if point else 0.0
                    s.angle_increment = 0.01
                    s.ranges = [float(math.hypot(*point)) if point else math.inf]
                    s.angle_max = s.angle_min
                    scan.publish(s)
                if send_command:
                    v = Twist()
                    v.linear.x = requested
                    cmd.publish(v)
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            deadline = time.monotonic() + 10
            while node.count_subscribers("/scan") == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert node.count_subscribers("/scan")
            pump(0.5)
            assert status[-1]["reason"] == "clear" and output[-1].linear.x == 0.3, status[-1]
            pump(0.5, point=(0.35, 0))
            assert (
                status[-1]["reason"] == "braking_slowdown" and 0 < output[-1].linear.x < 0.3
            ), status[-1]
            pump(0.5, point=(0.241, 0))
            assert (
                status[-1]["reason"] == "braking_slowdown" and 0 < output[-1].linear.x < 0.3
            ), status[-1]
            pump(0.5, point=(0.10, 0))
            assert (
                status[-1]["reason"] == "measured_braking_collision" and output[-1].linear.x == 0
            ), status[-1]

            pump(0.8, point=None, scan_min=0.30)
            assert (
                status[-1]["reason"] == "measured_braking_collision" and output[-1].linear.x == 0
            ), status[-1]
            pump(0.5, point=(0.36, 0), actual=0.35)
            assert (
                status[-1]["reason"] == "measured_braking_collision" and output[-1].linear.x == 0
            ), status[-1]
            pump(0.5)
            pump(0.4, send_command=False)
            assert status[-1]["reason"] == "command_stale" and output[-1].linear.x == 0, status[-1]
            pump(0.5)
            pump(0.4, send_scan=False)
            assert status[-1]["reason"] == "scan_or_tf_stale" and output[-1].linear.x == 0, status[
                -1
            ]
            pump(0.5)
            pump(0.3, send_odom=False)
            assert status[-1]["reason"] == "odometry_stale" and output[-1].linear.x == 0, status[-1]
            pump(0.5, requested=math.nan)
            assert status[-1]["reason"] == "invalid_command" and output[-1].linear.x == 0, status[
                -1
            ]
            pump(0.5)
            m = OccupancyGrid()
            m.header.frame_id = "map"
            m.header.stamp = node.get_clock().now().to_msg()
            m.info.width = m.info.height = 1
            m.info.resolution = 0.01
            m.info.origin.position.x = 0.50
            m.info.origin.orientation.w = 1.0
            m.data = [100]
            prior.publish(m)
            c = PoseWithCovarianceStamped()
            c.pose.covariance[0] = c.pose.covariance[7] = 0.25
            covariance.publish(c)
            pump(0.3)
            assert status[-1]["reason"] == "clear" and output[-1].linear.x == 0.3, status[-1]

            c.pose.covariance[0] = math.nan
            covariance.publish(c)
            generated = OccupancyGrid()
            generated.header.frame_id = "odom"
            generated.info.origin.position.x = math.nan
            grid.publish(generated)
            pump(0.75)
            assert status[-1]["reason"] == "clear" and output[-1].linear.x == 0.3, status[-1]
            assert (
                node.count_subscribers("/amcl_pose")
                == node.count_subscribers("/local_costmap/costmap")
                == 0
            )
            m.header.stamp = node.get_clock().now().to_msg()
            m.info.origin.position.x = 0.10
            prior.publish(m)
            pump(0.3)
            assert (
                status[-1]["reason"] == "measured_braking_collision" and output[-1].linear.x == 0
            ), status[-1]
            print(
                json.dumps(
                    {
                        "checks": 13,
                        "filled_scan_interior": True,
                        "filled_map_interior": True,
                        "braking_slowdown": True,
                        "core_validated_boundary_speed": True,
                        "quality_estimates_do_not_inflate_geometry": True,
                        "generated_costmap_not_a_guard_input": True,
                        "blind_obstacle_memory": True,
                        "positive_free_ray_clearing": True,
                        "measured_speed_braking": True,
                        "command_dropout": True,
                        "scan_dropout": True,
                        "odometry_dropout": True,
                        "invalid_command": True,
                    }
                )
            )
        finally:
            os.killpg(process.pid, signal.SIGINT)
            returncode = process.wait(timeout=5)
            assert returncode == 0, returncode
            print(json.dumps({"cleanup": {"returncode": returncode, "forced": False}}))
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    run()
