#!/usr/bin/env python3
import os
import select
import termios
import threading
import time
import tty

import geometry_msgs.msg
import rcl_interfaces.msg
import rclpy

from duojin01_teleop.keyboard_teleop_core import KeyboardTeleopCore
from duojin01_teleop.keyboard_teleop_defaults import (
    ACCEL_LIMIT_ANGULAR,
    ACCEL_LIMIT_LINEAR,
    DECEL_LIMIT_ANGULAR,
    DECEL_LIMIT_LINEAR,
    DEFAULT_IDLE_TIMEOUT_SEC,
    keyboard_teleop_parameter_defaults,
)

MSG = """
当前按键布局：
   Q    W    E
   A    S    D
        X

W/S : 前进/后退 (线速度 x)
A/D : 左移/右移 (线速度 y)
Q/E : 左转/右转 (角速度 z)
X   : 紧急停止

I/K : 线速度 增加/减少（固定步长）
O/L : 角速度 增加/减少（固定步长）

速度变化采用平滑加减速
TTY 模式通过按键重复超时推断松开

CTRL-C 退出
"""

IDLE_TIMEOUT_SEC = DEFAULT_IDLE_TIMEOUT_SEC


def vels(linear_speed, angular_speed):
    return '当前速度:\t线速度 %.2f m/s\t角速度 %.2f rad/s' % (linear_speed, angular_speed)


def resolve_tty_device_path(explicit_device_path: str) -> str:
    candidates = []
    if explicit_device_path:
        candidates.append(explicit_device_path)

    try:
        candidates.append(os.ttyname(0))
    except OSError:
        pass

    candidates.append('/dev/tty')

    seen = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)

        fd = -1
        try:
            fd = os.open(candidate, os.O_RDONLY | os.O_NONBLOCK)
            if os.isatty(fd):
                return candidate
        except OSError:
            continue
        finally:
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass

    raise RuntimeError('No interactive tty available')


class CmdVelPublisherThread(threading.Thread):
    def __init__(self, node, publisher, twist_msg, twist, stamped, core, rate_hz, stop_event, stale_key_timeout_sec):
        super().__init__(daemon=True)
        self.node = node
        self.publisher = publisher
        self.twist_msg = twist_msg
        self.twist = twist
        self.stamped = stamped
        self.core = core
        self.stop_event = stop_event
        self.period = 1.0 / rate_hz if rate_hz > 0.0 else 0.01
        self.stale_key_timeout_sec = stale_key_timeout_sec

    def _publish_command(self, command):
        if self.stamped:
            self.twist_msg.header.stamp = self.node.get_clock().now().to_msg()

        self.twist.linear.x = command.linear_x
        self.twist.linear.y = command.linear_y
        self.twist.linear.z = command.linear_z
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = command.angular_z
        self.publisher.publish(self.twist_msg)

    def run(self):
        while rclpy.ok() and not self.stop_event.is_set():
            self._publish_command(self.core.snapshot(stale_key_timeout_sec=self.stale_key_timeout_sec))
            self.stop_event.wait(self.period)

        self.core.emergency_stop()
        self._publish_command(self.core.zero_command())


class TtyKeyboardThread(threading.Thread):
    def __init__(self, node, core, stop_event, tty_device_path, read_poll_timeout):
        super().__init__(daemon=True)
        self.node = node
        self.core = core
        self.stop_event = stop_event
        self.tty_device_path = str(tty_device_path)
        self.read_poll_timeout = max(0.001, float(read_poll_timeout))

    def _handle_input_char(self, char_code, now):
        if char_code == 3:
            self.stop_event.set()
            return

        key = chr(char_code).lower()
        if self.core.handle_key_press(key, now) and self.core.is_speed_key(key):
            status = self.core.status()
            print(vels(status.linear_speed, status.angular_speed))

    def run(self):
        tty_fd = -1
        old_termios = None

        try:
            tty_path = resolve_tty_device_path(self.tty_device_path)
            tty_fd = os.open(tty_path, os.O_RDONLY | os.O_NONBLOCK)
            old_termios = termios.tcgetattr(tty_fd)
            tty.setraw(tty_fd)
            self.node.get_logger().info(f'Keyboard tty teleop attached to {tty_path}')

            while rclpy.ok() and not self.stop_event.is_set():
                ready_fds, _, _ = select.select([tty_fd], [], [], self.read_poll_timeout)
                if not ready_fds:
                    continue

                try:
                    chunk = os.read(tty_fd, 64)
                except BlockingIOError:
                    continue
                except OSError:
                    continue

                if not chunk:
                    continue

                now = time.monotonic()
                for char_code in chunk:
                    self._handle_input_char(char_code, now)
                    if self.stop_event.is_set():
                        break
        except Exception as exception:
            self.node.get_logger().error(f'TTY keyboard thread failed: {exception}')
            self.stop_event.set()
        finally:
            self.core.clear_move_keys(time.monotonic())
            if tty_fd >= 0 and old_termios is not None:
                try:
                    termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_termios)
                except OSError:
                    pass
            if tty_fd >= 0:
                try:
                    os.close(tty_fd)
                except OSError:
                    pass


def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop_node')

    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    parameters = {
        name: node.declare_parameter(name, default, read_only_descriptor).value
        for name, default in keyboard_teleop_parameter_defaults().items()
    }
    stamped = parameters['stamped']
    frame_id = parameters['frame_id']
    speed = parameters['speed']
    turn = parameters['turn']
    max_speed = parameters['max_speed']
    max_turn = parameters['max_turn']
    speed_step = parameters['speed_step']
    turn_step = parameters['turn_step']
    cmd_vel_topic = parameters['cmd_vel_topic']
    publish_rate = parameters['publish_rate']
    repeat_timeout_sec = parameters['repeat_timeout_sec']
    read_poll_timeout = parameters['read_poll_timeout']
    tty_device_path = parameters['tty_device_path']

    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        twist_msg_type = geometry_msgs.msg.TwistStamped
    else:
        twist_msg_type = geometry_msgs.msg.Twist

    publisher = node.create_publisher(twist_msg_type, cmd_vel_topic, 10)
    twist_msg = twist_msg_type()
    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    stop_event = threading.Event()
    core = KeyboardTeleopCore(
        linear_speed=float(speed),
        angular_speed=float(turn),
        speed_step=float(speed_step),
        turn_step=float(turn_step),
        max_linear_speed=float(max_speed),
        max_angular_speed=float(max_turn),
        accel_limit_linear=ACCEL_LIMIT_LINEAR,
        decel_limit_linear=DECEL_LIMIT_LINEAR,
        accel_limit_angular=ACCEL_LIMIT_ANGULAR,
        decel_limit_angular=DECEL_LIMIT_ANGULAR,
        idle_timeout_sec=IDLE_TIMEOUT_SEC,
    )

    publisher_thread = CmdVelPublisherThread(
        node=node,
        publisher=publisher,
        twist_msg=twist_msg,
        twist=twist,
        stamped=stamped,
        core=core,
        rate_hz=float(publish_rate),
        stop_event=stop_event,
        stale_key_timeout_sec=max(0.0, float(repeat_timeout_sec)),
    )

    keyboard_thread = TtyKeyboardThread(
        node=node,
        core=core,
        stop_event=stop_event,
        tty_device_path=tty_device_path,
        read_poll_timeout=read_poll_timeout,
    )

    try:
        resolve_tty_device_path(str(tty_device_path))
        print(MSG)
        print('平滑限制:\t线加速 %.2f m/s²\t线减速 %.2f m/s²\t角加速 %.2f rad/s²\t角减速 %.2f rad/s²' % (
            ACCEL_LIMIT_LINEAR,
            DECEL_LIMIT_LINEAR,
            ACCEL_LIMIT_ANGULAR,
            DECEL_LIMIT_ANGULAR,
        ))
        print('TTY 模式参数:\t按键超时 %.2f s\t轮询超时 %.3f s' % (
            max(0.0, float(repeat_timeout_sec)),
            max(0.001, float(read_poll_timeout)),
        ))
        print('速度上限:\t线速度 %.2f m/s\t角速度 %.2f rad/s' % (
            float(max_speed),
            float(max_turn),
        ))
        print(vels(float(speed), float(turn)))
        publisher_thread.start()
        keyboard_thread.start()

        while rclpy.ok() and not stop_event.is_set():
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    except Exception as exception:
        node.get_logger().error(f'Keyboard tty teleop failed to start: {exception}')
    finally:
        stop_event.set()
        core.emergency_stop()
        if keyboard_thread.is_alive():
            keyboard_thread.join(timeout=1.0)
        if publisher_thread.is_alive():
            publisher_thread.join(timeout=1.0)
        rclpy.shutdown()
        spinner.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
