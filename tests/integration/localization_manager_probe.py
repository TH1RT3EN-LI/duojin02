"""Exercise the fixed-map command owner and the complete PTY command chain.

Uses the installed Humble APIs, real DDS and compiled mux/guard/serial driver.
The circular map is a controlled sensor fixture, not simulator ground truth.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from ros_isolation import configure_isolation


ROOT = Path(__file__).resolve().parents[2]


class SensorFixture(Node):
    def __init__(self):
        super().__init__('localization_owner_probe', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', durable)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', durable)
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.command_pub = self.create_publisher(Twist, '/nav/localization_requested', 10)
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

    def pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.orientation.w = 1.0
        return pose

    def pump(self, duration, *, scan=True, invalid=False, nav=None, manual=None, pose=False):
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
                    message.ranges = [2.0]*540
                    if invalid:
                        message.ranges[7] = math.nan
                    self.scan_pub.publish(message)
                    self.scan_count += 1
                    if pose:
                        self.pose_pub.publish(self.pose())
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


def run(args):
    domain = configure_isolation()
    rclpy.init()
    fixture = SensorFixture()
    processes, logs = [], []
    result = {'pass': False, 'checks': [], 'cleanup': [], 'ros_domain_id': domain,
              'scope': 'DDS, controlled scan/map, compiled mux/guard, PTY driver; no Gazebo or physical robot'}

    def start(name, command):
        log = (args.output_dir/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))
        return process

    def check(name, condition):
        assert condition, {'case': name, 'status': fixture.status[-1] if fixture.status else None,
                           'output': fixture.outputs[-3:], 'wire': fixture.wire_packets[-3:],
                           'guard': fixture.guard_status[-1:]}
        result['checks'].append(name)

    args.output_dir.mkdir(parents=True, exist_ok=False)
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    fixture.master = master
    try:
        manager = start('manager', ['ros2', 'run', 'duojin01_bringup', 'localization_manager',
            '--ros-args', '--params-file', str(ROOT/'src/duojin01_bringup/config/localization_health.yaml'),
            '-p', 'use_sim_time:=true'])
        start('mux', ['ros2','run','twist_mux','twist_mux','--ros-args',
            '--params-file',str(ROOT/'src/duojin01_bringup/config/twist_mux.yaml'),
            '-r','/cmd_vel_out:=/cmd_vel_requested'])
        start('guard', [str(args.guard_binary),'--ros-args','--params-file',
            str(ROOT/'src/duojin01_bringup/config/velocity_guard.yaml'), '-p','use_sim_time:=true'])
        start('driver', [str(args.driver_binary),'--ros-args','-p',f'usart_port_name:={os.ttyname(slave)}',
            '-p','serial_baud_rate:=57600','-p','use_sim_time:=true','-r','/cmd_vel:=/cmd_vel_base'])
        fixture.pump(.3, scan=False, nav=.24)
        check('startup_closed', not any(x != 0 for _,x in fixture.outputs))
        fixture.pump(2.5, nav=.24)
        check('real_map_scan_health', bool(fixture.status) and fixture.status[-1][1]['healthy'])
        check('fresh_navigation_forwarded', bool(fixture.outputs) and fixture.outputs[-1][1] == .24)
        check('canonical_guard_topic', bool(fixture.base) and fixture.base[-1][1] == .24)
        check('actual_serial_motion', bool(fixture.wire_packets) and fixture.wire_packets[-1][1] == (240,0,0))
        fixture.pump(.45)
        check('command_expiry_zero', fixture.outputs[-1][1] == 0)
        count = len(fixture.outputs)
        fixture.pump(.7, manual=.12)
        check('idle_releases_navigation_lease', len(fixture.outputs) == count)
        check('manual_takeover_reaches_serial', fixture.wire_packets[-1][1] == (120,0,0))
        fixture.pump(.7, nav=.24)
        fixture.pump(.6, scan=False, nav=.24)
        check('scan_loss_closes_navigation', not fixture.status[-1][1]['healthy'] and fixture.outputs[-1][1] == 0)
        start_index = len(fixture.outputs)
        fixture.pump(1.0)
        check('health_requalified_without_command_replay', fixture.status[-1][1]['healthy'] and
              not any(x != 0 for _,x in fixture.outputs[start_index:]))
        fixture.pump(.6, nav=.24)
        fixture.clock_on = False
        fixture.pump(.65, scan=False, nav=.24)
        check('paused_clock_wall_watchdog', not fixture.status[-1][1]['healthy'] and fixture.outputs[-1][1] == 0)
        fixture.clock_on = True
        fixture.pump(1.0)
        fixture.pump(.6, nav=.24)
        fixture.pump(.3, invalid=True, nav=.24)
        check('malformed_scan_latches_closed', not fixture.status[-1][1]['healthy'] and fixture.outputs[-1][1] == 0)
        fixture.pump(.8, nav=.24)
        check('normal_scans_do_not_clear_latched_fault', not fixture.status[-1][1]['healthy'])
        fixture.initial_pub.publish(fixture.pose())
        start_index = len(fixture.outputs)
        fixture.pump(2.0, pose=True)
        check('explicit_recovery_requires_new_command', fixture.status[-1][1]['healthy'] and
              not any(x != 0 for _,x in fixture.outputs[start_index:]))
        fixture.pump(.6, nav=.24)
        check('post_recovery_request_reaches_serial', fixture.wire_packets[-1][1] == (240,0,0))
        # An actual DDS sample waits while the owner is stopped. Its source
        # timestamp must keep it from executing when the owner resumes.
        os.killpg(manager.pid, signal.SIGSTOP)
        command = Twist();command.linear.x = .31;fixture.command_pub.publish(command)
        fixture.pump(.7)
        check('owner_loss_stops_serial_independently', fixture.wire_packets[-1][1] == (0,0,0))
        start_index = len(fixture.outputs)
        os.killpg(manager.pid, signal.SIGCONT)
        fixture.pump(1.2)
        check('queued_dds_command_never_revives', fixture.status[-1][1]['healthy'] and
              not any(x != 0 for _,x in fixture.outputs[start_index:]))
        fixture.pump(.6, nav=.24)
        check('fresh_request_after_owner_pause', fixture.wire_packets[-1][1] == (240,0,0))
        result['pass'] = True
    finally:
        for name, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGCONT)
                os.killpg(process.pid, signal.SIGINT)
            returncode = process.wait(timeout=8)
            result['cleanup'].append({'name': name, 'returncode': returncode, 'forced': False})
            result['pass'] = result['pass'] and returncode == 0
        result['published'] = {'clock':fixture.clock_count, 'scans':fixture.scan_count, 'commands':fixture.command_count}
        result['observed'] = {'health':len(fixture.status), 'commands':len(fixture.outputs),
                              'guard':len(fixture.base), 'serial_packets':len(fixture.wire_packets)}
        result['final_status'] = fixture.status[-1][1] if fixture.status else None
        (args.output_dir/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        fixture.close()
        rclpy.try_shutdown()
        os.close(master);os.close(slave)
        for log in logs:
            log.close()
    assert result['pass'], result['cleanup']
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--guard-binary', type=Path, required=True)
    parser.add_argument('--driver-binary', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    run(parser.parse_args())
