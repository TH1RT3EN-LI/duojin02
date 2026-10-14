from copy import deepcopy
import math
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
import yaml

from test_teleop_input_lifecycle import load_node

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((ROOT / "src/duojin02_teleop/config/joy_launcher.yaml").read_text())[
    "joy_launcher_node"
]["ros__parameters"]


class FakeNode:
    overrides = {}
    instances = []

    def __init__(self, name):
        self.parameters = {**deepcopy(CONFIG), **deepcopy(self.overrides)}
        self.parameters.setdefault("command_timeout_s", 0.25)
        self.feedback = []
        self.subscriptions = []
        self.timers = []
        self.clock_ns = 100_000_000_000
        self.instances.append(self)

    def declare_parameter(self, name, default, descriptor):
        self.parameters.setdefault(name, [] if default in ("strings", "integers") else default)

    def get_parameter(self, name):
        value = self.parameters[name]
        return SimpleNamespace(
            value=value,
            get_parameter_value=lambda: SimpleNamespace(
                string_array_value=value, integer_array_value=value
            ),
        )

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None, warning=lambda message: None)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.clock_ns))

    def create_subscription(self, *args):
        self.subscriptions.append(args)

    def create_publisher(self, *args):
        return SimpleNamespace(
            publish=lambda message: self.feedback.append((message.id, message.intensity))
        )

    def create_timer(self, *args):
        self.timers.append(args)


@pytest.fixture
def make_launcher(monkeypatch):
    module = load_node(monkeypatch, "joy_launcher_node.py", FakeNode)
    module.rclpy.Parameter = SimpleNamespace(
        Type=SimpleNamespace(STRING_ARRAY="strings", INTEGER_ARRAY="integers", DOUBLE=None)
    )
    module.JoyFeedback.TYPE_RUMBLE = 1
    clock = SimpleNamespace(wall=100.0)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: clock.wall))
    spawned = []

    def spawn(command, **kwargs):
        process = SimpleNamespace(code=None)
        process.poll = lambda: process.code
        spawned.append((command, process))
        return process

    monkeypatch.setattr(
        module,
        "subprocess",
        SimpleNamespace(
            Popen=spawn, DEVNULL=subprocess.DEVNULL, TimeoutExpired=subprocess.TimeoutExpired
        ),
    )

    def create(**overrides):
        FakeNode.overrides = overrides
        FakeNode.instances = []
        node = module.JoyLauncherNode()

        def feed(wall, pressed=False, *, length=8, stamp_ns=None, value=1):
            clock.wall = wall
            node.clock_ns = round(wall * 1e9)
            stamp_ns = node.clock_ns if stamp_ns is None else stamp_ns
            buttons = [0] * length
            if pressed and length > 3:
                buttons[3] = value
            node.joy_callback(
                SimpleNamespace(
                    buttons=buttons,
                    header=SimpleNamespace(
                        stamp=SimpleNamespace(
                            sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000
                        )
                    ),
                )
            )

        return node, feed, spawned

    return create


def test_continuous_long_press_fires_once_and_duplicate_inflight_is_ignored(make_launcher):
    node, feed, spawned = make_launcher()
    feed(100.0)
    for index in range(1, 46):
        feed(100.0 + index * 0.05, True)
    assert len(spawned) == 1
    feed(102.3)
    for index in range(1, 46):
        feed(102.3 + index * 0.05, True)
    assert len(spawned) == 1
    spawned[0][1].code = 0
    node._poll_running_processes()
    feed(104.6, True)
    assert len(spawned) == 1
    feed(104.7)
    for index in range(1, 46):
        feed(104.7 + index * 0.05, True)
    assert len(spawned) == 2


def test_gap_resets_hold_start_without_replaying_old_hold(make_launcher):
    node, feed, spawned = make_launcher()
    feed(100.0)
    feed(100.05, True)
    feed(103.0, True)
    assert not spawned
    assert node.button_press_start_ts[3] == 103.0
    for index in range(1, 42):
        feed(103.0 + index * 0.05, True)
    assert len(spawned) == 1


@pytest.mark.parametrize("replay_stamp", [100_050_000_000, 100_000_000_000])
def test_duplicate_or_backward_stamp_does_not_renew_input(make_launcher, replay_stamp):
    node, feed, spawned = make_launcher()
    feed(100.0)
    feed(100.05, True)
    for index in range(1, 50):
        feed(100.05 + index * 0.05, True, stamp_ns=replay_stamp)
    assert node.last_joy_receipt == 100.05
    assert not spawned
    feed(102.6, True)
    assert node.button_press_start_ts[3] == 102.6


@pytest.mark.parametrize("length,value", [(2, 1), (8, 2)])
def test_missing_or_invalid_button_clears_hold_state(make_launcher, length, value):
    node, feed, spawned = make_launcher()
    feed(100.0)
    feed(100.05, True)
    for index in range(1, 45):
        feed(100.05 + index * 0.05, True, length=length, value=value)
    assert 3 not in node.button_press_start_ts
    feed(102.4, True)
    assert node.button_press_start_ts[3] == 102.4
    assert not spawned


def test_invalid_buttons_clear_hold_even_on_duplicate_header(make_launcher):
    node, feed, _ = make_launcher()
    feed(100.0)
    feed(100.05, True)
    feed(100.1, True, length=2, stamp_ns=100_050_000_000)
    assert 3 not in node.button_press_start_ts
    feed(100.15, True)
    assert node.button_press_start_ts[3] == 100.15


@pytest.mark.parametrize("hold", ["3:nan", "3:inf", "3:-1", "3:bad", "-1:2", "4:2"])
def test_invalid_hold_is_rejected_instead_of_repaired_or_falling_back_to_zero(make_launcher, hold):
    with pytest.raises(ValueError, match="hold_bindings"):
        make_launcher(hold_bindings=[hold])
    assert not FakeNode.instances[-1].subscriptions


@pytest.mark.parametrize(
    "name,value",
    [
        ("command_timeout_s", 0.0),
        ("command_timeout_s", math.nan),
        ("default_hold_sec", math.nan),
        ("default_hold_sec", -1.0),
        ("trigger_feedback_duration_sec", math.nan),
        ("trigger_feedback_duration_sec", -1.0),
        ("success_feedback_duration_sec", math.inf),
        ("trigger_feedback_intensity", math.nan),
        ("trigger_feedback_intensity", 1.1),
        ("feedback_stop_check_period_sec", 0.0),
        ("process_poll_period_sec", math.inf),
    ],
)
def test_invalid_numeric_configuration_fails_before_subscription(make_launcher, name, value):
    with pytest.raises(ValueError, match=name):
        make_launcher(**{name: value})
    assert not FakeNode.instances[-1].subscriptions


def test_missing_canonical_lease_is_a_named_error(make_launcher):
    with pytest.raises(ValueError, match="command_timeout_s"):
        make_launcher(command_timeout_s=None)


def test_feedback_uses_supported_single_channel_and_stops_on_deadline(make_launcher):
    node, feed, _ = make_launcher()
    node._publish_rumble(0.45, 0.12)
    assert node.feedback == [(0, 0.45)]
    feed(100.13)
    node._feedback_timer_callback()
    assert node.feedback == [(0, 0.45), (0, 0.0)]


def test_failed_process_does_not_publish_success_feedback(make_launcher):
    node, feed, spawned = make_launcher()
    feed(100.0)
    for index in range(1, 46):
        feed(100.0 + index * 0.05, True)
    before = list(node.feedback)
    spawned[0][1].code = 1
    node._poll_running_processes()
    assert node.feedback == before
    assert not node.running_procs
