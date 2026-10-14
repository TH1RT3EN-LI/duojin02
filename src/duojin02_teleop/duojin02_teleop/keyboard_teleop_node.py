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

from duojin02_teleop.keyboard_teleop_core import KeyboardTeleopCore
from duojin02_teleop.keyboard_teleop_defaults import (
    ACCEL_LIMIT_ANGULAR,
    ACCEL_LIMIT_LINEAR,
    DECEL_LIMIT_ANGULAR,
    DECEL_LIMIT_LINEAR,
    keyboard_teleop_parameter_defaults,
    validate_keyboard_parameters,
)

MSG = """
当前按键布局：
   Q    W    E
   A    S    D
        X

W/S : 前进/后退 (线速度 x)
A/D : 左移/右移 (线速度 y)
Q/E : 左转/右转 (角速度 z)
X   : 当前键盘指令立即归零

I/K : 线速度 增加/减少（固定步长）
O/L : 角速度 增加/减少（固定步长）

速度变化采用平滑加减速
TTY 模式通过按键重复超时推断松开；松开后交回导航

CTRL-C 退出
"""


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
    def __init__(self, publisher, core, rate_hz, stop_event, stale_key_timeout_sec):
        super().__init__(daemon=True)
        self.publisher = publisher
        self.twist = geometry_msgs.msg.Twist()
        self.core = core
        self.stop_event = stop_event
        self.period = 1.0 / rate_hz
        self.stale_key_timeout_sec = stale_key_timeout_sec
        self.was_active = False

    def _publish_command(self, command):
        self.twist.linear.x = command.linear_x
        self.twist.linear.y = command.linear_y
        self.twist.angular.z = command.angular_z
        self.publisher.publish(self.twist)

    def _publish_if_active(self, command):
        # Held opposing keys still mean the operator owns the input, at zero speed.
        active = bool(command.active_keys) or any((
            command.linear_x, command.linear_y, command.angular_z,
        ))
        if active or self.was_active or command.stop_requested:
            self._publish_command(command)
        self.was_active = active

    def run(self):
        try:
            while rclpy.ok() and not self.stop_event.is_set():
                self._publish_if_active(self.core.snapshot(
                    stale_key_timeout_sec=self.stale_key_timeout_sec))
                self.stop_event.wait(self.period)
        finally:
            self.core.emergency_stop()
            command = self.core.zero_command()
            if self.was_active or command.stop_requested:
                self._publish_command(command)
                self.was_active = False


class TtyKeyboardThread(threading.Thread):
    def __init__(self, node, core, stop_event, tty_device_path, read_poll_timeout):
        super().__init__(daemon=True)
        self.node = node
        self.core = core
        self.stop_event = stop_event
        self.tty_device_path = str(tty_device_path)
        self.read_poll_timeout = float(read_poll_timeout)

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
                if not chunk:
                    raise RuntimeError('Keyboard tty closed')

                now = time.monotonic()
                for char_code in chunk:
                    self._handle_input_char(char_code, now)
                    if self.stop_event.is_set():
                        break
        except Exception as exception:
            self.node.get_logger().error(f'TTY keyboard thread failed: {exception}')
            self.stop_event.set()
        finally:
            self.core.emergency_stop(time.monotonic())
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
    node = None
    core = None
    spinner = publisher_thread = keyboard_thread = None
    stop_event = threading.Event()
    try:
        node = rclpy.create_node('keyboard_teleop_node')
        read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
        parameters = {
            name: node.declare_parameter(name, default, read_only_descriptor).value
            for name, default in keyboard_teleop_parameter_defaults().items()
        }
        validate_keyboard_parameters(parameters)
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
        )
        publisher = node.create_publisher(geometry_msgs.msg.Twist, cmd_vel_topic, 10)
        publisher_thread = CmdVelPublisherThread(
            publisher=publisher,
            core=core,
            rate_hz=float(publish_rate),
            stop_event=stop_event,
            stale_key_timeout_sec=float(repeat_timeout_sec),
        )
        keyboard_thread = TtyKeyboardThread(
            node=node,
            core=core,
            stop_event=stop_event,
            tty_device_path=tty_device_path,
            read_poll_timeout=read_poll_timeout,
        )

        resolve_tty_device_path(str(tty_device_path))
        print(MSG)
        print('平滑限制:\t线加速 %.2f m/s²\t线减速 %.2f m/s²\t角加速 %.2f rad/s²\t角减速 %.2f rad/s²' % (
            ACCEL_LIMIT_LINEAR,
            DECEL_LIMIT_LINEAR,
            ACCEL_LIMIT_ANGULAR,
            DECEL_LIMIT_ANGULAR,
        ))
        print('TTY 模式参数:\t按键超时 %.2f s\t轮询超时 %.3f s' % (
            float(repeat_timeout_sec),
            float(read_poll_timeout),
        ))
        print('速度上限:\t线速度 %.2f m/s\t角速度 %.2f rad/s' % (
            float(max_speed),
            float(max_turn),
        ))
        print(vels(float(speed), float(turn)))
        spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spinner.start()
        publisher_thread.start()
        keyboard_thread.start()

        while rclpy.ok() and not stop_event.is_set():
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    except Exception as exception:
        if node is not None:
            node.get_logger().error(f'Keyboard tty teleop failed: {exception}')
        raise
    finally:
        stop_event.set()
        if core is not None:
            core.emergency_stop()
        for worker in (keyboard_thread, publisher_thread):
            if worker is not None and worker.is_alive():
                worker.join(timeout=1.0)
        rclpy.try_shutdown()
        if spinner is not None:
            spinner.join(timeout=1.0)
        if node is not None:
            node.destroy_node()


if __name__ == '__main__':
    main()
