"""Exercise the compiled serial driver against a PTY; never opens physical devices."""
import json
import os
import signal
import struct
import subprocess
import time
from pathlib import Path
from ament_index_python.packages import get_package_prefix

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


def frame(vx=1.0, gyro=100):
    data = bytearray([0x7b, 0])
    data.extend(struct.pack('>3h', int(vx * 1000), 0, 0))
    data.extend(struct.pack('>6h', 0, 0, 16384, 0, 0, gyro))
    data.extend(struct.pack('>h', 24000))
    checksum = 0
    for byte in data:
        checksum ^= byte
    return bytes(data + bytearray([checksum, 0x7d]))


def run():
    os.environ['ROS_DOMAIN_ID'] = '91'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    master, slave = os.openpty()
    path = os.ttyname(slave)
    rclpy.init()
    node = Node('base_feedback_probe')
    odometry, inertial = [], []
    node.create_subscription(Odometry, '/odom', odometry.append, 100)
    node.create_subscription(Imu, '/imu', inertial.append, 100)
    commands = node.create_publisher(Twist, '/cmd_vel', 10)
    logfile = '/tmp/base-feedback-probe.log'
    with open(logfile, 'w') as log:
        process = subprocess.Popen([
            str(Path(get_package_prefix('duojin02_base_driver'))/'lib/duojin02_base_driver/duojin02_base_driver_node'), '--ros-args',
            '-p', f'usart_port_name:={path}', '-p', 'serial_baud_rate:=57600',
            '-p', 'loop_hz:=200', '-p', 'imu_yaw_rate_bias:=0.01',
            '-p', 'imu_yaw_rate_variance:=0.02',
            '-p', 'odom_twist_variances_moving:=[0.01, 0.02, 1e6, 1e6, 1e6, 0.03]'
        ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            # Wait for DDS discovery with no valid feedback, then send at 10 Hz.
            deadline = time.monotonic() + 10
            while node.count_publishers('/odom') == 0 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert node.count_publishers('/odom'), f'No driver publisher; see {logfile}'
            for index in range(26):
                os.write(master, frame())
                until = time.monotonic() + 0.1
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.005)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.01)
            assert process.poll() is None, f'Driver stopped; see {logfile}'
            before_bad_frame = len(odometry)
            bad = bytearray(frame(vx=4.0))
            bad[22] ^= 0xff
            os.write(master, bad)
            until = time.monotonic() + 0.1
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.005)
            assert len(odometry) == before_bad_frame, 'Bad checksum produced odometry'
            assert len(odometry) >= 24, len(odometry)
            def stamp(message):
                return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            elapsed = stamp(odometry[-1]) - stamp(odometry[0])
            distance = odometry[-1].pose.pose.position.x - odometry[0].pose.pose.position.x
            assert abs(distance - elapsed) < 0.04, (elapsed, distance)
            assert abs(odometry[-1].twist.covariance[7] - 0.02) < 1e-9
            assert abs(odometry[-1].twist.covariance[35] - 0.03) < 1e-9
            assert inertial
            assert abs(inertial[-1].angular_velocity.z - (100 * 0.00026644 - 0.01)) < 1e-6
            assert inertial[-1].angular_velocity_covariance[8] == 0.02

            # Check the actual wire bytes, including int16 saturation before rounding.
            os.set_blocking(master, False)
            def read_pending():
                try:
                    return os.read(master, 4096)
                except BlockingIOError:
                    return b''
            read_pending()
            for values, expected in [
                ((1.234, -0.5, 0.75), (1234, -500, 750)),
                ((1e100, -1e100, 32.768), (32767, -32768, 32767)),
                ((float('nan'), float('inf'), -float('inf')), (0, 0, 0)),
            ]:
                command = Twist()
                command.linear.x, command.linear.y, command.angular.z = values
                commands.publish(command)
                wire = b''
                deadline = time.monotonic() + 2
                while len(wire) < 11 and time.monotonic() < deadline:
                    os.write(master, frame(vx=0.0))
                    rclpy.spin_once(node, timeout_sec=0.02)
                    wire += read_pending()
                assert len(wire) == 11, wire
                assert wire[0] == 0x7b and wire[-1] == 0x7d, wire
                assert struct.unpack('>3h', wire[3:9]) == expected, wire
                checksum = 0
                for byte in wire[:9]:
                    checksum ^= byte
                assert wire[9] == checksum, wire

            # Stop the command publisher while feedback stays healthy: the driver
            # must send an actual zero frame if the upstream command source stops.
            read_pending();command=Twist();command.linear.x=0.3;commands.publish(command)
            published=time.monotonic();wire=b'';expired_stop=None;saw_motion=False
            while time.monotonic()-published<.6 and expired_stop is None:
                os.write(master,frame(vx=0.0));rclpy.spin_once(node,timeout_sec=.01)
                wire+=read_pending()
                while len(wire)>=11:
                    packet,wire=wire[:11],wire[11:]
                    velocity=struct.unpack('>3h',packet[3:9])
                    if velocity==(300,0,0):saw_motion=True
                    if saw_motion and velocity==(0,0,0):expired_stop=time.monotonic()-published
            assert expired_stop is not None and .23<=expired_stop<.4, expired_stop

            # Silence forces close/reopen; the first new frame must not integrate the gap.
            until = time.monotonic() + 0.85
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.01)
            position_before = odometry[-1].pose.pose.position.x
            before_reconnect = len(odometry)
            for _ in range(8):
                os.write(master, frame())
                until = time.monotonic() + 0.1
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.005)
            assert len(odometry) > before_reconnect, 'Feedback did not recover'
            assert abs(odometry[before_reconnect].pose.pose.position.x - position_before) < 0.03
            print(json.dumps({'feedback_hz': 10, 'timer_hz': 200, 'samples': len(odometry),
                              'elapsed_seconds': elapsed, 'distance_meters': distance,
                              'ratio': distance / elapsed, 'custom_baud': 57600,
                              'bad_checksum_ignored': True, 'calibration_parameters': 'passed',
                              'command_encoding': 'passed', 'command_expiry_stop_s':expired_stop,
                              'silent_reconnect': 'passed'}, indent=2))
        finally:
            os.killpg(process.pid, signal.SIGINT)
            returncode = process.wait(timeout=10)
            assert returncode == 0, returncode
            print(json.dumps({'cleanup':{'returncode':returncode,'forced':False}}))
            node.destroy_node()
            rclpy.shutdown()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    run()
