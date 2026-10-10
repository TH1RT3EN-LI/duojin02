"""Simulated device endpoints matching the unchanged hardware ROS contract."""
import math
import json
import struct
import random
import signal
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Imu
from std_msgs.msg import Float32, Float64, Int8
from duojin02_sim.geometry import inside_convex


class DeviceAdapter(Node):
    def __init__(self):
        super().__init__('sim_devices')
        self.timeout = self.declare_parameter('command_timeout_s', 0.25).value
        self.radius = self.declare_parameter('wheel_radius_m', 0.0503669839553).value
        self.k = (self.declare_parameter('wheelbase_m', 0.312755710719).value +
                  self.declare_parameter('track_m', 0.355082035065).value) / 2
        self.range_max = self.declare_parameter('published_range_max_m', 100.0).value
        self.range_min = self.declare_parameter('published_range_min_m', 0.3).value
        self.self_filter = self.declare_parameter('self_filter', True).value
        self.footprint = json.loads(self.declare_parameter('footprint', '[]').value)
        self.laser_xy = self.declare_parameter('laser_xy', [0.0, 0.0]).value
        if self.self_filter and len(self.footprint) < 3:
            raise ValueError('self_filter requires the inherited chassis footprint')
        self.motor_running = True
        self.command, self.last_command = Twist(), None
        self.last_scan_stamp = None
        self.odom_scale = self.declare_parameter('odom_velocity_scale', 1.0).value
        self.yaw_bias = self.declare_parameter('gyro_bias_radps', 0.0).value
        self.odom_noise = self.declare_parameter('odom_velocity_noise_mps', 0.0).value
        self.rng = random.Random(self.declare_parameter('random_seed', 1).value)
        self.cmd_pub = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.wheels = [self.create_publisher(Float64, '/sim/wheels/' + n, 10) for n in
                       ('front_left', 'front_right', 'rear_left', 'rear_right')]
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/lslidar_point_cloud', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.battery_pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.create_subscription(Twist, '/cmd_vel_base', self.command_cb, 10)
        self.create_subscription(Int8, '/motor_control', self.motor_cb, 1)
        self.create_subscription(LaserScan, '/sim/lidar/raw', self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/ground_truth/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/sim/imu/raw', self.imu_cb, qos_profile_sensor_data)
        self.create_timer(0.02, self.drive)
        self.create_timer(1.0, lambda: self.battery_pub.publish(Float32(data=24.0)))
        self.get_logger().info('KINEMATIC base; GPU nearest-return lidar; /odom pose is ideal, twist uses configured errors; battery fixed at 24 V')

    def command_cb(self, msg):
        self.command = msg
        self.last_command = self.get_clock().now().nanoseconds

    def drive(self):
        now = self.get_clock().now().nanoseconds
        command = self.command if self.last_command is not None and 0 <= now-self.last_command <= self.timeout*1e9 else Twist()
        planar = Twist()
        planar.linear.x, planar.linear.y = command.linear.x, command.linear.y
        planar.angular.z = command.angular.z
        self.cmd_pub.publish(planar)
        x, y, w = planar.linear.x, planar.linear.y, self.k*planar.angular.z
        for pub, speed in zip(self.wheels, (x-y-w, x+y+w, x+y-w, x-y+w)):
            pub.publish(Float64(data=speed/self.radius))

    def motor_cb(self, msg):
        if msg.data in (0, 1):
            self.motor_running = bool(msg.data)
            self.last_scan_stamp = None

    def scan_cb(self, msg):
        if not self.motor_running:
            return
        msg.header.frame_id = 'laser'
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        msg.scan_time = max(0.0, stamp-self.last_scan_stamp) if self.last_scan_stamp is not None else 0.1
        self.last_scan_stamp = stamp
        msg.time_increment = 0.0  # hardware angle-sorted bins have no per-ray timing
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        ranges, intensities, points = [], [], bytearray()
        for i, value in enumerate(msg.ranges):
            angle = msg.angle_min + i*msg.angle_increment
            valid = math.isfinite(value) and msg.range_min <= value <= self.range_max
            if valid and self.self_filter:
                valid = not inside_convex(self.laser_xy[0]+value*math.cos(angle),
                                          self.laser_xy[1]+value*math.sin(angle), self.footprint)
            ranges.append(value if valid else math.inf)
            intensities.append(1.0 if valid else math.nan)
            if valid:
                points.extend(struct.pack('<ffff', value*math.cos(angle), value*math.sin(angle), 0.0, 1.0))
        msg.ranges, msg.intensities = ranges, intensities
        self.scan_pub.publish(msg)
        cloud = PointCloud2(header=msg.header, height=1, width=len(points)//16,
                            fields=[PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                    for i, n in enumerate(('x', 'y', 'z', 'intensity'))],
                            is_bigendian=False, point_step=16, row_step=len(points),
                            data=bytes(points), is_dense=True)
        self.cloud_pub.publish(cloud)

    def odom_cb(self, msg):
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_footprint'
        msg.twist.twist.linear.x = msg.twist.twist.linear.x*self.odom_scale+self.rng.gauss(0,self.odom_noise)
        msg.twist.twist.linear.y = msg.twist.twist.linear.y*self.odom_scale+self.rng.gauss(0,self.odom_noise)
        msg.twist.twist.angular.z += self.yaw_bias
        for i in (0, 7, 35):
            msg.pose.covariance[i], msg.twist.covariance[i] = 0.0001, 0.0004
        self.odom_pub.publish(msg)

    def imu_cb(self, msg):
        msg.header.frame_id = 'imu_link'
        msg.angular_velocity.z += self.yaw_bias
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = 0.0001
            msg.angular_velocity_covariance[i] = 0.0004
            msg.linear_acceleration_covariance[i] = 0.0025
        self.imu_pub.publish(msg)


def main():
    # The device owner sends its final zero while DDS is still alive, then
    # closes its context. rclpy's default signal handler closes it first.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DeviceAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    running = True
    def stop(signum, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while running:
            executor.spin_once(timeout_sec=.05)
    finally:
        node.cmd_pub.publish(Twist())
        executor.shutdown()
        rclpy.shutdown()
        node.destroy_node()
