import math
import os
import signal
import subprocess
import time
from typing import Dict, Set

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Joy, JoyFeedback


class JoyLauncherNode(Node):
    def __init__(self):
        super().__init__("joy_launcher_node")

        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter("command_timeout_s", rclpy.Parameter.Type.DOUBLE, descriptor)
        self.command_timeout_s = self._number("command_timeout_s", positive=True)
        self.declare_parameter("bindings", rclpy.Parameter.Type.STRING_ARRAY, descriptor)
        self.declare_parameter("hold_bindings", rclpy.Parameter.Type.STRING_ARRAY, descriptor)
        self.declare_parameter("default_hold_sec", 0.0, descriptor)
        self.declare_parameter(
            "trigger_feedback_buttons",
            rclpy.Parameter.Type.INTEGER_ARRAY,
            descriptor,
        )
        self.declare_parameter(
            "success_feedback_buttons",
            rclpy.Parameter.Type.INTEGER_ARRAY,
            descriptor,
        )
        self.declare_parameter("feedback_topic", "/joy/set_feedback", descriptor)
        self.declare_parameter("trigger_feedback_intensity", 0.45, descriptor)
        self.declare_parameter("trigger_feedback_duration_sec", 0.15, descriptor)
        self.declare_parameter("success_feedback_intensity", 1.0, descriptor)
        self.declare_parameter("success_feedback_duration_sec", 0.35, descriptor)
        self.declare_parameter("feedback_stop_check_period_sec", 0.02, descriptor)
        self.declare_parameter("process_poll_period_sec", 0.10, descriptor)

        bindings = self._get_string_array("bindings")
        hold_bindings = self._get_string_array("hold_bindings")

        self.button_commands = self._parse_button_commands(
            bindings,
            "bindings",
        )
        self.button_hold_secs = self._parse_button_floats(
            hold_bindings,
            "hold_bindings",
        )
        self.default_hold_sec = self._number("default_hold_sec")
        if set(self.button_hold_secs) - set(self.button_commands):
            raise ValueError("hold_bindings must refer to configured bindings")

        self.trigger_feedback_buttons = self._parse_button_set("trigger_feedback_buttons")
        self.success_feedback_buttons = self._parse_button_set("success_feedback_buttons")

        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.trigger_feedback_intensity = self._number("trigger_feedback_intensity", maximum=1.0)
        self.success_feedback_intensity = self._number("success_feedback_intensity", maximum=1.0)
        self.trigger_feedback_duration_sec = self._number("trigger_feedback_duration_sec")
        self.success_feedback_duration_sec = self._number("success_feedback_duration_sec")
        self.feedback_stop_check_period_sec = self._number(
            "feedback_stop_check_period_sec", positive=True
        )
        self.process_poll_period_sec = self._number("process_poll_period_sec", positive=True)

        self.prev_buttons = []
        self.running_procs: Dict[int, subprocess.Popen] = {}
        self.button_press_start_ts: Dict[int, float] = {}
        self.button_fired_while_held: Set[int] = set()
        self.feedback_stop_deadline = None
        self.last_joy_stamp_ns = -1
        self.last_joy_receipt = None

        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.feedback_pub = self.create_publisher(
            JoyFeedback,
            self.feedback_topic,
            10,
        )
        self.create_timer(
            self.process_poll_period_sec,
            self._poll_running_processes,
        )
        self.create_timer(
            self.feedback_stop_check_period_sec,
            self._feedback_timer_callback,
        )

        self.get_logger().info(
            f"JoyLauncher ready, bindings:{self.button_commands}, "
            f"hold_secs:{self.button_hold_secs}, "
            f"default_hold_sec:{self.default_hold_sec:.2f}"
        )
        self.get_logger().info(
            f"Feedback topic:{self.feedback_topic}, "
            f"trigger_buttons:{sorted(self.trigger_feedback_buttons)}, "
            f"success_buttons:{sorted(self.success_feedback_buttons)}"
        )

    def _number(self, name, *, positive=False, maximum=None):
        try:
            value = float(self.get_parameter(name).value)
        except (TypeError, ValueError, ParameterUninitializedException) as error:
            raise ValueError(f"{name} must be supplied with a supported numeric value") from error
        if (
            not math.isfinite(value)
            or value < 0
            or (positive and value == 0)
            or (maximum is not None and value > maximum)
        ):
            raise ValueError(f"{name} must be finite and in its supported range")
        return value

    def _parse_button_commands(self, raw_bindings, param_name):
        parsed = {}
        for entry in raw_bindings:
            try:
                button_str, command = entry.split(":", 1)
                button = int(button_str)
                if button < 0 or button in parsed or not command.strip():
                    raise ValueError()
                parsed[button] = command
            except ValueError as error:
                raise ValueError(f"Invalid {param_name} entry {entry!r}") from error
        return parsed

    def _parse_button_floats(self, raw_bindings, param_name):
        parsed = {}
        for entry in raw_bindings:
            try:
                button_str, value_str = entry.split(":", 1)
                button, value = int(button_str), float(value_str)
                if button < 0 or button in parsed or not math.isfinite(value) or value < 0:
                    raise ValueError()
                parsed[button] = value
            except ValueError as error:
                raise ValueError(f"Invalid {param_name} entry {entry!r}") from error
        return parsed

    def _parse_button_set(self, param_name):
        raw_values = self._get_integer_array(param_name)
        if any(button < 0 for button in raw_values):
            raise ValueError(f"{param_name} must contain nonnegative button indices")
        return set(raw_values)

    def _get_string_array(self, param_name):
        try:
            value = self.get_parameter(param_name).get_parameter_value()
            return list(value.string_array_value)
        except ParameterUninitializedException:
            return []

    def _get_integer_array(self, param_name):
        try:
            value = self.get_parameter(param_name).get_parameter_value()
            return list(value.integer_array_value)
        except ParameterUninitializedException:
            return []

    def _hold_seconds_for_button(self, button_idx):
        return self.button_hold_secs.get(button_idx, self.default_hold_sec)

    def _reset_hold_state(self):
        self.prev_buttons = []
        self.button_press_start_ts.clear()
        self.button_fired_while_held.clear()

    def _clear_button(self, button):
        self.button_press_start_ts.pop(button, None)
        self.button_fired_while_held.discard(button)
        if button < len(self.prev_buttons):
            self.prev_buttons[button] = 0

    def joy_callback(self, msg: Joy):
        invalid_buttons = {
            button
            for button in self.button_commands
            if button >= len(msg.buttons) or msg.buttons[button] not in (0, 1)
        }
        for button in invalid_buttons:
            self._clear_button(button)
        now = time.monotonic()
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (
            stamp.sec < 0
            or not 0 <= stamp.nanosec < 1_000_000_000
            or stamp_ns <= 0
            or abs(self.get_clock().now().nanoseconds - stamp_ns) >= self.command_timeout_s * 1e9
        ):
            self._reset_hold_state()
            return
        if stamp_ns <= self.last_joy_stamp_ns:
            return
        if self.last_joy_receipt is None or now - self.last_joy_receipt >= self.command_timeout_s:
            self._reset_hold_state()
        self.last_joy_stamp_ns, self.last_joy_receipt = stamp_ns, now

        for btn, cmd in self.button_commands.items():
            if btn in invalid_buttons:
                continue
            current_pressed = msg.buttons[btn] == 1
            prev_pressed = btn < len(self.prev_buttons) and self.prev_buttons[btn] == 1
            if current_pressed and not prev_pressed:
                self.button_press_start_ts[btn] = now
                self.button_fired_while_held.discard(btn)
            if current_pressed and btn not in self.button_fired_while_held:
                hold_sec = self._hold_seconds_for_button(btn)
                held_sec = now - self.button_press_start_ts.get(btn, now)
                if held_sec >= hold_sec:
                    self._trigger_button_command(btn, cmd, hold_sec, held_sec)
                    self.button_fired_while_held.add(btn)
            if not current_pressed:
                self.button_press_start_ts.pop(btn, None)
                self.button_fired_while_held.discard(btn)
        self.prev_buttons = list(msg.buttons)

    def _trigger_button_command(self, btn, cmd, hold_sec, held_sec):
        if btn in self.running_procs and self.running_procs[btn].poll() is None:
            self.get_logger().info(f"Command for button {btn} is still running; ignoring repeat")
            return

        if hold_sec > 0.0:
            self.get_logger().info(
                f"Button {btn} long-press {held_sec:.2f}s/{hold_sec:.2f}s -> {cmd}"
            )
        else:
            self.get_logger().info(f"Button {btn} pressed -> {cmd}")

        if btn in self.trigger_feedback_buttons:
            self._publish_rumble(
                self.trigger_feedback_intensity,
                self.trigger_feedback_duration_sec,
            )

        self.running_procs[btn] = subprocess.Popen(
            ["bash", "-lc", cmd],
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _poll_running_processes(self):
        for btn, proc in list(self.running_procs.items()):
            return_code = proc.poll()
            if return_code is None:
                continue

            if return_code == 0:
                self.get_logger().info(f"Command for button {btn} finished successfully")
                if btn in self.success_feedback_buttons:
                    self._publish_rumble(
                        self.success_feedback_intensity,
                        self.success_feedback_duration_sec,
                    )
            else:
                self.get_logger().warning(
                    f"Command for button {btn} exited with code {return_code}"
                )

            self.running_procs.pop(btn, None)

    def _stop_process_tree(self, btn, proc):
        if proc.poll() is not None:
            return

        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return

        try:
            proc.wait(timeout=1.0)
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                f"Process group for button {btn} did not exit after SIGTERM, sending SIGKILL"
            )

        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            return

        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(f"Process group for button {btn} still running after SIGKILL")

    def _publish_rumble(self, intensity, duration_sec):
        if not rclpy.ok():
            return

        feedback = JoyFeedback()
        feedback.type = JoyFeedback.TYPE_RUMBLE
        feedback.id = 0
        feedback.intensity = float(intensity)
        try:
            self.feedback_pub.publish(feedback)
        except RuntimeError:
            return

        if duration_sec > 0.0:
            self.feedback_stop_deadline = time.monotonic() + duration_sec
        else:
            self.feedback_stop_deadline = None

    def _feedback_timer_callback(self):
        if self.feedback_stop_deadline is None:
            return

        if time.monotonic() >= self.feedback_stop_deadline:
            self.feedback_stop_deadline = None
            self._publish_rumble(0.0, 0.0)

    def destroy_node(self):
        for btn, proc in list(self.running_procs.items()):
            if proc.poll() is None:
                self.get_logger().info(f"Stopping process for button {btn}")
                self._stop_process_tree(btn, proc)
        self._publish_rumble(0.0, 0.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JoyLauncherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
