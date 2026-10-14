import os
import signal
import subprocess
import time
from typing import Dict, Set

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from sensor_msgs.msg import Joy, JoyFeedback

class JoyLauncherNode(Node):
    def __init__(self):
        super().__init__("joy_launcher_node")

        self.declare_parameter('bindings', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('hold_bindings', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('default_hold_sec', 0.0)
        self.declare_parameter(
            'trigger_feedback_buttons',
            rclpy.Parameter.Type.INTEGER_ARRAY,
        )
        self.declare_parameter(
            'success_feedback_buttons',
            rclpy.Parameter.Type.INTEGER_ARRAY,
        )
        self.declare_parameter('feedback_topic', '/joy/set_feedback')
        self.declare_parameter('trigger_feedback_intensity', 0.45)
        self.declare_parameter('trigger_feedback_duration_sec', 0.15)
        self.declare_parameter('success_feedback_intensity', 1.0)
        self.declare_parameter('success_feedback_duration_sec', 0.35)
        self.declare_parameter('feedback_stop_check_period_sec', 0.02)
        self.declare_parameter('process_poll_period_sec', 0.10)

        bindings = self._get_string_array('bindings')
        hold_bindings = self._get_string_array('hold_bindings')

        self.button_commands = self._parse_button_commands(
            bindings,
            'bindings',
        )
        self.button_hold_secs = self._parse_button_floats(
            hold_bindings,
            'hold_bindings',
        )
        self.default_hold_sec = max(
            0.0,
            float(self.get_parameter('default_hold_sec').value),
        )

        self.trigger_feedback_buttons = self._parse_button_set(
            'trigger_feedback_buttons'
        )
        self.success_feedback_buttons = self._parse_button_set(
            'success_feedback_buttons'
        )

        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.trigger_feedback_intensity = self._clamp_intensity(
            float(self.get_parameter('trigger_feedback_intensity').value)
        )
        self.success_feedback_intensity = self._clamp_intensity(
            float(self.get_parameter('success_feedback_intensity').value)
        )
        self.trigger_feedback_duration_sec = max(
            0.0,
            float(self.get_parameter('trigger_feedback_duration_sec').value),
        )
        self.success_feedback_duration_sec = max(
            0.0,
            float(self.get_parameter('success_feedback_duration_sec').value),
        )
        self.feedback_stop_check_period_sec = max(
            0.01,
            float(self.get_parameter('feedback_stop_check_period_sec').value),
        )
        self.process_poll_period_sec = max(
            0.05,
            float(self.get_parameter('process_poll_period_sec').value),
        )

        self.prev_buttons = []
        self.running_procs: Dict[int, subprocess.Popen] = {}
        self.button_press_start_ts: Dict[int, float] = {}
        self.button_fired_while_held: Set[int] = set()
        self.feedback_stop_deadline = None

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
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

    def _parse_button_commands(self, raw_bindings, param_name):
        parsed = {}
        for entry in raw_bindings:
            try:
                button_str, command = entry.split(':', 1)
                parsed[int(button_str)] = command
            except ValueError:
                self.get_logger().warning(
                    f'Invalid {param_name} entry "{entry}", expected "button:command".'
                )
        return parsed

    def _parse_button_floats(self, raw_bindings, param_name):
        parsed = {}
        for entry in raw_bindings:
            try:
                button_str, value_str = entry.split(':', 1)
                parsed[int(button_str)] = max(0.0, float(value_str))
            except ValueError:
                self.get_logger().warning(
                    f'Invalid {param_name} entry "{entry}", expected "button:value".'
                )
        return parsed

    def _parse_button_set(self, param_name):
        raw_values = self._get_integer_array(param_name)
        return {int(button) for button in raw_values}

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

    def _clamp_intensity(self, intensity):
        return max(0.0, min(1.0, intensity))

    def joy_callback(self, msg: Joy):
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
            return

        now = time.monotonic()
        for btn, cmd in self.button_commands.items():
            if btn < len(msg.buttons) and btn < len(self.prev_buttons):
                current_pressed = msg.buttons[btn] == 1
                prev_pressed = self.prev_buttons[btn] == 1

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
        if hold_sec > 0.0:
            self.get_logger().info(
                f'Button {btn} long-press {held_sec:.2f}s/{hold_sec:.2f}s -> {cmd}'
            )
        else:
            self.get_logger().info(f'Button {btn} pressed -> {cmd}')

        if btn in self.trigger_feedback_buttons:
            self._publish_rumble(
                self.trigger_feedback_intensity,
                self.trigger_feedback_duration_sec,
            )

        if btn in self.running_procs and self.running_procs[btn].poll() is None:
            self.get_logger().info(f'Killing previous process for button {btn}')
            self._stop_process_tree(btn, self.running_procs[btn])

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
                self.get_logger().info(f'Command for button {btn} finished successfully')
                if btn in self.success_feedback_buttons:
                    self._publish_rumble(
                        self.success_feedback_intensity,
                        self.success_feedback_duration_sec,
                    )
            else:
                self.get_logger().warning(
                    f'Command for button {btn} exited with code {return_code}'
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
                f'Process group for button {btn} did not exit after SIGTERM, sending SIGKILL'
            )

        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            return

        try:
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                f'Process group for button {btn} still running after SIGKILL'
            )

    def _publish_rumble(self, intensity, duration_sec):
        if not rclpy.ok():
            return

        rumble_intensity = self._clamp_intensity(float(intensity))

        for rumble_id in (0, 1):
            feedback = JoyFeedback()
            feedback.type = JoyFeedback.TYPE_RUMBLE
            feedback.id = rumble_id
            feedback.intensity = rumble_intensity
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
    node = JoyLauncherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
