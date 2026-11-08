#!/usr/bin/env python3
"""Exercise both startup modes in a ROS domain without a robot or navigation."""

import os
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class Inputs(Node):
    def __init__(self):
        super().__init__("readiness_modes_probe_inputs")
        self.scans = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.contracts = self.create_publisher(DiagnosticArray, "amcl_estimate_contract", 10)
        self.transforms = TransformBroadcaster(self)

    def publish(self, *, scan=True, tf=True, amcl=False):
        stamp = self.get_clock().now().to_msg()
        if tf:
            transforms = []
            for parent, child in (("map", "odom"), ("odom", "base_footprint"),
                                  ("base_footprint", "laser")):
                item = TransformStamped()
                item.header.stamp, item.header.frame_id = stamp, parent
                item.child_frame_id = child
                item.transform.rotation.w = 1.0
                transforms.append(item)
            self.transforms.sendTransform(transforms)
        if scan:
            msg = LaserScan()
            msg.header.stamp, msg.header.frame_id = stamp, "laser"
            msg.angle_min, msg.angle_max, msg.angle_increment = -1.0, 1.0, 0.1
            msg.scan_time, msg.time_increment = 0.1, 0.0
            msg.range_min, msg.range_max, msg.ranges = 0.02, 10.0, [2.0] * 21
            self.scans.publish(msg)
        if amcl:
            status = DiagnosticStatus(name="amcl/estimate_contract")
            status.values = [KeyValue(key=k, value=v) for k, v in {
                "global_acquisition_pending": "false",
                "first_pf_confirmation_pending": "false",
                "valid_observation": "true",
            }.items()]
            self.contracts.publish(DiagnosticArray(status=[status]))


def exercise(node, label, parameters, blocked_inputs, ready_inputs):
    args = [sys.executable, "-m", "duojin02_bringup.localization_readiness", "--ros-args"]
    for key, value in parameters.items():
        args += ["-p", f"{key}:={value}"]
    child = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"))
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            node.publish(**blocked_inputs)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.045)
            if child.poll() is not None:
                raise AssertionError(f"{label}: started despite missing input: {child.communicate()[0]}")
        deadline = time.monotonic() + 5.0
        while child.poll() is None and time.monotonic() < deadline:
            node.publish(**ready_inputs)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.045)
        if child.poll() is None:
            raise AssertionError(f"{label}: did not become ready")
        output = child.communicate(timeout=1)[0]
        if child.returncode != 0:
            raise AssertionError(f"{label}: exited {child.returncode}: {output}")
        print(f"PASS {label}: missing input held startup; valid input released it", flush=True)
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=3)


def main():
    rclpy.init()
    node = Inputs()
    try:
        exercise(node, "fixed map requires AMCL", {},
                 {"scan": True, "tf": True, "amcl": False},
                 {"scan": True, "tf": True, "amcl": True})
        exercise(node, "online SLAM requires scan without AMCL",
                 {"require_amcl_estimate": "false"},
                 {"scan": False, "tf": True, "amcl": False},
                 {"scan": True, "tf": True, "amcl": False})
        exercise(node, "online SLAM requires transforms without AMCL",
                 {"require_amcl_estimate": "false"},
                 {"scan": True, "tf": False, "amcl": False},
                 {"scan": True, "tf": True, "amcl": False})
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
