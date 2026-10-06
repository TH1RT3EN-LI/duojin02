"""Run the installed lidar executable against a PTY and observe standard ROS output."""
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


def packet(start_angle, distance):
    data = bytearray(108)
    data[:3] = bytes([0xa5, 0x5a, 108])
    def write16(offset, value):
        data[offset:offset + 2] = value.to_bytes(2, 'big')
    write16(5, start_angle)
    write16(105, (start_angle + 1500) % 36000)
    for group in range(16):
        for echo in range(2):
            write16(7 + group * 6 + echo * 3, distance)
            data[9 + group * 6 + echo * 3] = 40 + echo
    data[-1] = sum(data[:-1]) & 0xff
    return data


def run():
    os.environ.update(ROS_DOMAIN_ID='95', ROS_LOCALHOST_ONLY='1')
    master, slave = os.openpty()
    executable = Path(get_package_prefix('lslidar_driver')) / 'lib/lslidar_driver/lslidar_driver_node'
    rclpy.init()
    node = Node('lidar_output_probe')
    scans, clouds = [], []
    node.create_subscription(LaserScan, '/scan', scans.append, 100)
    node.create_subscription(PointCloud2, '/lslidar_point_cloud', clouds.append, 100)
    logfile = '/tmp/lidar-output-probe.log'
    with open(logfile, 'w') as log:
        process = subprocess.Popen([
            str(executable), '--ros-args', '-p', f'serial_port:={os.ttyname(slave)}',
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.count_publishers('/scan') and node.count_publishers('/lslidar_point_cloud'):
                    break
                assert process.poll() is None, f'Node exited; see {logfile}'
            assert node.count_publishers('/scan'), f'No scan publisher; see {logfile}'
            for turn in range(7):
                distance = 0 if 3 <= turn <= 5 else 1000
                for group in range(24):
                    os.write(master, packet(group * 1500, distance))
                    rclpy.spin_once(node, timeout_sec=0.0042)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and (len(scans) < 6 or len(clouds) < 6):
                rclpy.spin_once(node, timeout_sec=0.02)
            assert len(scans) == len(clouds) == 6, (len(scans), len(clouds), logfile)
            def stamp(message):
                return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
            by_stamp = {stamp(cloud): cloud for cloud in clouds}
            assert all(stamp(b) > stamp(a) for a, b in zip(scans, scans[1:]))
            empty = 0
            for scan in scans:
                assert scan.header.frame_id == 'laser' and len(scan.ranges) == 540
                assert scan.time_increment == 0.0
                cloud = by_stamp[stamp(scan)]
                assert cloud.header.frame_id == scan.header.frame_id
                if all(math.isinf(value) for value in scan.ranges):
                    empty += 1
                    assert cloud.width == 0
                else:
                    assert cloud.width > 0
                    assert abs(scan.ranges[270] - 1.0) < 1e-5
            # The first zero-distance turn shares a boundary point with its predecessor.
            assert empty == 2, empty
            # Actual disconnection must end the installed node with a failure result.
            os.close(master)
            master = None
            assert process.wait(timeout=10) == 1, f'Disconnect not reported; see {logfile}'
            print(json.dumps({'installed_node': str(executable), 'paired_frames': len(scans),
                              'empty_frames': empty, 'disconnect_exit': 1}, indent=2))
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()
            if master is not None:
                os.close(master)
            os.close(slave)


if __name__ == '__main__':
    run()
