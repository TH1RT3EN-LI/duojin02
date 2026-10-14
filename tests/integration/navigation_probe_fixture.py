"""Controlled sensor, DDS and PTY fixture for navigation command-chain probes."""
import json
import math
import os
from pathlib import Path
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

ROOT = Path(__file__).resolve().parents[2]


class SensorFixture(Node):
    def __init__(self):
        super().__init__('navigation_probe_fixture', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', durable)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.manual_pub = self.create_publisher(Twist, '/cmd_vel_normal', 10)
        self.status, self.outputs, self.base, self.requested, self.guard_status = [], [], [], [], []
        self.create_subscription(String, '/localization/status',
            lambda m: self.status.append((time.monotonic(), json.loads(m.data))), durable)
        self.create_subscription(String, '/navigation/guard_status',
            lambda m: self.guard_status.append(json.loads(m.data)), 10)
        for topic, target in [('/cmd_vel', self.outputs), ('/cmd_vel_base', self.base),
                              ('/cmd_vel_requested', self.requested)]:
            self.create_subscription(Twist, topic,
                lambda m, target=target: target.append((time.monotonic(), m.linear.x)), 10)
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for parent, child in [('odom', 'laser'), ('odom', 'base_footprint')]:
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        self.map_tf = TransformBroadcaster(self)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 500
        grid.info.resolution = .01
        grid.info.origin.position.x = grid.info.origin.position.y = -2.5
        grid.info.origin.orientation.w = 1.0
        grid.data = [100 if 1.98 <= math.hypot(x*.01-2.5, y*.01-2.5) <= 2.02 else 0
                     for y in range(500) for x in range(500)]
        self.map_pub.publish(grid)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.running, self.clock_on = True, True
        self.clock_count, self.scan_count, self.command_count = 0, 0, 0
        self.source_start = time.monotonic()
        self.clock_thread = threading.Thread(target=self.publish_clock)
        self.clock_thread.start()
        self.next_scan = self.next_command = self.next_odom = self.next_feedback = 0.0
        self.master = None
        self.wire, self.wire_packets = b'', []

    def publish_clock(self):
        next_clock = time.monotonic()
        while self.running:
            if self.clock_on:
                now = 100.0 + time.monotonic() - self.source_start
                message = Clock()
                message.clock.sec = int(now)
                message.clock.nanosec = int((now-int(now))*1e9)
                self.clock_pub.publish(message)
                self.clock_count += 1
            next_clock += .001
            time.sleep(max(0.0, next_clock-time.monotonic()))

    def pump(self, duration, *, scan=True, invalid=False, nav=None, manual=None, range_m=2.0):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            now = time.monotonic()
            stamp = self.get_clock().now().to_msg()
            if now >= self.next_scan:
                self.next_scan = now+.1
                if scan:
                    message = LaserScan()
                    message.header.frame_id, message.header.stamp = 'laser', stamp
                    message.range_min, message.range_max = .02, 8.0
                    message.angle_min, message.angle_increment = -math.pi, 2*math.pi/540
                    message.angle_max = -math.pi+539*message.angle_increment
                    message.scan_time = .1
                    message.ranges = [range_m]*540
                    if invalid:
                        message.ranges = [math.nan]*540
                    self.scan_pub.publish(message)
                    self.scan_count += 1
            if now >= self.next_command:
                self.next_command = now+.05
                for value, publisher in [(nav, self.command_pub), (manual, self.manual_pub)]:
                    if value is not None:
                        command = Twist()
                        command.linear.x = value
                        publisher.publish(command)
                        if publisher is self.command_pub:
                            self.command_count += 1
            if now >= self.next_odom:
                self.next_odom = now+.05
                odom = Odometry()
                odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
                odom.header.stamp, odom.pose.pose.orientation.w = stamp, 1.0
                self.odom_pub.publish(odom)
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'map', 'odom'
                # This controlled identity transform remains valid ahead of
                # the scan, as AMCL's transform_tolerance does in production.
                # Cross-topic DDS arrival order must not make scans untransformable.
                transform.header.stamp = (self.get_clock().now()+rclpy.duration.Duration(seconds=.1)).to_msg()
                transform.transform.rotation.w = 1.0
                self.map_tf.sendTransform(transform)
            if self.master is not None:
                if now >= self.next_feedback:
                    self.next_feedback = now+.1
                    feedback = bytearray([0x7b, 0])+bytearray(struct.pack('>9h', 0,0,0,0,0,16384,0,0,0))
                    feedback.extend(struct.pack('>h', 24000))
                    checksum = 0
                    for byte in feedback:
                        checksum ^= byte
                    os.write(self.master, feedback+bytes([checksum,0x7d]))
                try:
                    self.wire += os.read(self.master, 4096)
                except BlockingIOError:
                    pass
                while len(self.wire) >= 11:
                    packet, self.wire = self.wire[:11], self.wire[11:]
                    checksum = 0
                    for byte in packet[:9]:
                        checksum ^= byte
                    assert packet[0] == 0x7b and packet[10] == 0x7d and packet[9] == checksum
                    self.wire_packets.append((time.monotonic(), struct.unpack('>3h', packet[3:9])))
            rclpy.spin_once(self, timeout_sec=.001)

    def close(self):
        self.running = False
        self.clock_thread.join(timeout=2)
        assert not self.clock_thread.is_alive()
        self.destroy_node()

