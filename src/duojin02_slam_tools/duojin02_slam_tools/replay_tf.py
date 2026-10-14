"""Relay only the transforms owned by the selected recorded-data odometry route."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan


def retained(transforms, odom_source, map_frame="map", odom_frame="odom", base_frame="base_footprint"):
    def keep(transform):
        parent = transform.header.frame_id.lstrip("/")
        child = transform.child_frame_id.lstrip("/")
        if child == map_frame or parent == map_frame:
            return False
        if odom_source == "ekf" and child in (odom_frame, base_frame):
            return False
        return True
    return [transform for transform in transforms if keep(transform)]


class ReplayTfFilter(Node):
    def __init__(self):
        super().__init__("replay_tf_filter")
        self.source = self.declare_parameter("odom_source", "recorded").value
        if self.source not in ("recorded", "ekf"):
            raise ValueError("odom_source must be recorded or ekf")
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.dynamic = self.create_publisher(TFMessage, "/tf", reliable)
        self.static = self.create_publisher(TFMessage, "/tf_static", latched)
        self.static_transforms = {}
        self.scan_start = self.declare_parameter("scan_start_time_ns", 0).value
        sensor = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan = self.create_publisher(LaserScan, "/scan", reliable)
        self.create_subscription(LaserScan, "/replay/scan_in", self.scan_callback, sensor)
        self.create_subscription(TFMessage, "/replay/tf_in", self.dynamic_callback, reliable)
        # The relay starts before playback, so volatile input also accepts legacy bags.
        self.create_subscription(TFMessage, "/replay/tf_static_in", self.static_callback, reliable)

    def dynamic_callback(self, message):
        transforms = retained(message.transforms, self.source)
        if transforms:
            self.dynamic.publish(TFMessage(transforms=transforms))

    def scan_callback(self, message):
        stamp = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if stamp >= self.scan_start:
            self.scan.publish(message)

    def static_callback(self, message):
        for transform in retained(message.transforms, self.source):
            self.static_transforms[transform.child_frame_id] = transform
        if self.static_transforms:
            self.static.publish(TFMessage(transforms=list(self.static_transforms.values())))


def main(args=None):
    rclpy.init(args=args)
    node = ReplayTfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
