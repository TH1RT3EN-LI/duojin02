"""Exercise input ownership and process lifetime without ROS or robot hardware."""

import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/duojin02_teleop'


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Publisher:
    def __init__(self):
        self.commands = []

    def publish(self, message):
        self.commands.append((message.linear.x, message.linear.y, message.angular.z))


def load_node(monkeypatch, filename):
    monkeypatch.syspath_prepend(str(PACKAGE))
    modules = {
        'geometry_msgs': {'msg': None},
        'geometry_msgs.msg': {'Twist': Twist},
        'rcl_interfaces': {'msg': None},
        'rcl_interfaces.msg': {},
        'rclpy': {'ok': lambda: True},
        'rclpy.node': {'Node': type('Node', (), {})},
        'rclpy.exceptions': {'ParameterUninitializedException': type('ParameterError', (Exception,), {})},
        'sensor_msgs': {'msg': None},
        'sensor_msgs.msg': {'Joy': type('Joy', (), {}), 'JoyFeedback': type('JoyFeedback', (), {})},
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    for name in ('geometry_msgs', 'rcl_interfaces', 'sensor_msgs'):
        sys.modules[name].msg = sys.modules[name + '.msg']
    path = PACKAGE / 'duojin02_teleop' / filename
    spec = importlib.util.spec_from_file_location('teleop_test_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_publisher(monkeypatch):
    node = load_node(monkeypatch, 'keyboard_teleop_node.py')
    core = node.KeyboardTeleopCore(
        linear_speed=1.0, angular_speed=1.0, speed_step=0.1, turn_step=0.1,
        accel_limit_linear=100.0, decel_limit_linear=100.0,
        accel_limit_angular=100.0, decel_limit_angular=100.0,
    )
    publisher = Publisher()
    thread = node.CmdVelPublisherThread(publisher, core, 120.0, threading.Event(), 0.5)
    return node, core, publisher, thread


def test_idle_and_speed_adjustment_do_not_publish_or_claim_motion(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    for timestamp in (100.0, 100.1, 100.2):
        thread._publish_if_active(core.snapshot(timestamp))
    core.handle_key_press('i', 100.3)
    thread._publish_if_active(core.snapshot(100.4))
    assert publisher.commands == []


def test_expired_motion_sends_one_final_zero_then_releases(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    thread._publish_if_active(core.snapshot(100.8))
    thread._publish_if_active(core.snapshot(100.9))
    thread._publish_if_active(core.snapshot(101.0))
    assert publisher.commands == [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]


def test_held_opposing_keys_continue_publishing_zero_until_expiry(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    core.handle_key_press('s', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    thread._publish_if_active(core.snapshot(100.2))
    thread._publish_if_active(core.snapshot(100.8))
    thread._publish_if_active(core.snapshot(100.9))
    assert publisher.commands == [(0.0, 0.0, 0.0)] * 3


def test_x_sends_one_explicit_zero_from_idle(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('x', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    thread._publish_if_active(core.snapshot(100.2))
    assert publisher.commands == [(0.0, 0.0, 0.0)]


def test_shutdown_clears_keys_and_sends_final_zero(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    thread.stop_event.set()
    thread.run()
    assert publisher.commands == [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    assert core.status().active_keys == []


def test_publisher_failure_still_attempts_final_zero(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    def broken_snapshot(**kwargs):
        raise RuntimeError('input snapshot failed')
    monkeypatch.setattr(core, 'snapshot', broken_snapshot)
    with pytest.raises(RuntimeError, match='input snapshot failed'):
        thread.run()
    assert publisher.commands == [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]


def test_idle_shutdown_does_not_claim_manual_priority(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    thread.stop_event.set()
    thread.run()
    assert publisher.commands == []


def test_shutdown_after_end_zero_does_not_publish_another_zero(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    thread._publish_if_active(core.snapshot(100.8))
    thread.stop_event.set()
    thread.run()
    assert publisher.commands == [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]


def test_shutdown_of_held_opposing_keys_sends_final_zero(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    core.handle_key_press('s', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    thread.stop_event.set()
    thread.run()
    assert publisher.commands == [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)]


@pytest.mark.parametrize('eof', [False, True])
def test_tty_failure_stops_input_and_restores_terminal(monkeypatch, eof):
    module, core, _, _ = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    restored, closed, errors = [], [], []
    logger = SimpleNamespace(info=lambda message: None, error=errors.append)
    node = SimpleNamespace(get_logger=lambda: logger)
    stop_event = threading.Event()
    thread = module.TtyKeyboardThread(node, core, stop_event, '/fake/tty', 0.02)
    monkeypatch.setattr(module, 'resolve_tty_device_path', lambda path: path)
    def read(fd, size):
        if eof:
            return b''
        raise OSError('tty disconnected')
    monkeypatch.setattr(module, 'os', SimpleNamespace(
        O_RDONLY=0, O_NONBLOCK=1, open=lambda *args: 42, read=read, close=closed.append))
    monkeypatch.setattr(module, 'termios', SimpleNamespace(
        TCSADRAIN=1, tcgetattr=lambda fd: 'settings',
        tcsetattr=lambda *args: restored.append(args)))
    monkeypatch.setattr(module, 'tty', SimpleNamespace(setraw=lambda fd: None))
    monkeypatch.setattr(module, 'select', SimpleNamespace(select=lambda *args: ([42], [], [])))
    thread.run()
    assert stop_event.is_set()
    assert core.status().active_keys == []
    command = core.snapshot()
    assert command.linear_x == command.linear_y == command.angular_z == 0.0
    assert restored == [(42, 1, 'settings')]
    assert closed == [42]
    assert len(errors) == 1


def test_duplicate_map_save_keeps_running_process_without_feedback_or_restart(monkeypatch):
    module = load_node(monkeypatch, 'joy_launcher_node.py')
    launcher = object.__new__(module.JoyLauncherNode)
    running = SimpleNamespace(poll=lambda: None)
    launcher.running_procs = {3: running}
    messages = []
    launcher.get_logger = lambda: SimpleNamespace(info=messages.append)
    def unexpected_call(*args, **kwargs):
        pytest.fail('Duplicate save must not stop, restart or rumble')
    launcher._stop_process_tree = unexpected_call
    launcher._publish_rumble = unexpected_call
    monkeypatch.setattr(module.subprocess, 'Popen', unexpected_call)
    launcher._trigger_button_command(3, 'save-map', 2.0, 2.1)
    assert launcher.running_procs[3] is running
    assert len(messages) == 1


def test_completed_map_save_can_be_started_again(monkeypatch):
    module = load_node(monkeypatch, 'joy_launcher_node.py')
    launcher = object.__new__(module.JoyLauncherNode)
    launcher.running_procs = {3: SimpleNamespace(poll=lambda: 0)}
    launcher.trigger_feedback_buttons = set()
    launcher.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    started = []
    process = object()
    def start(command, **kwargs):
        started.append(command)
        return process
    monkeypatch.setattr(module.subprocess, 'Popen', start)
    launcher._trigger_button_command(3, 'save-map', 2.0, 2.1)
    assert started == [['bash', '-lc', 'save-map']]
    assert launcher.running_procs[3] is process


def test_x_immediately_before_exit_still_publishes_explicit_zero(monkeypatch):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('x', 100.0)
    thread.stop_event.set()
    thread.run()
    assert publisher.commands == [(0.0, 0.0, 0.0)]


INVALID_STARTUP_VALUES = [
    (name, value)
    for name in ('repeat_timeout_sec', 'publish_rate', 'read_poll_timeout')
    for value in (0.0, -1.0, math.nan, math.inf, -math.inf)
] + [
    (name, value)
    for name in ('speed', 'turn', 'max_speed', 'max_turn', 'speed_step', 'turn_step')
    for value in (-1.0, math.nan, math.inf, -math.inf)
]


@pytest.mark.parametrize('name,value', INVALID_STARTUP_VALUES)
def test_invalid_startup_never_creates_command_publisher_or_spinner_and_cleans_context(
        monkeypatch, name, value):
    module = load_node(monkeypatch, 'keyboard_teleop_node.py')
    state = {'context_active': False, 'destroyed': False, 'errors': []}
    def unexpected_write(*args, **kwargs):
        pytest.fail('Invalid configuration must fail before command publisher or spinner creation')
    node = SimpleNamespace(
        declare_parameter=lambda parameter, default, descriptor: SimpleNamespace(
            value=value if parameter == name else default),
        create_publisher=unexpected_write,
        get_logger=lambda: SimpleNamespace(error=state['errors'].append),
        destroy_node=lambda: state.update(destroyed=True),
    )
    module.rcl_interfaces.msg.ParameterDescriptor = lambda **kwargs: SimpleNamespace(**kwargs)
    module.rclpy.init = lambda: state.update(context_active=True)
    module.rclpy.create_node = lambda name: node
    module.rclpy.spin = unexpected_write
    module.rclpy.try_shutdown = lambda: state.update(context_active=False)
    with pytest.raises(ValueError, match=name):
        module.main()
    assert state['destroyed']
    assert not state['context_active']
    assert len(state['errors']) == 1 and name in state['errors'][0]


def test_valid_parameter_values_are_not_repaired_or_retuned(monkeypatch):
    module = load_node(monkeypatch, 'keyboard_teleop_node.py')
    parameters = module.keyboard_teleop_parameter_defaults()
    parameters.update(speed=0.0, turn=0.0, max_speed=0.0, max_turn=0.0,
                      speed_step=0.0, turn_step=0.0, read_poll_timeout=.0001)
    original = dict(parameters)
    module.validate_keyboard_parameters(parameters)
    assert parameters == original
    thread = module.TtyKeyboardThread(None, None, threading.Event(), '', parameters['read_poll_timeout'])
    assert thread.read_poll_timeout == .0001


@pytest.mark.parametrize('timeout', [0.0, -1.0, math.nan, math.inf, -math.inf])
def test_invalid_runtime_expiry_causes_final_zero_instead_of_cached_repetition(monkeypatch, timeout):
    _, core, publisher, thread = make_publisher(monkeypatch)
    core.handle_key_press('w', 100.0)
    thread._publish_if_active(core.snapshot(100.1))
    thread.stale_key_timeout_sec = timeout
    with pytest.raises(ValueError, match='stale_key_timeout_sec'):
        thread.run()
    assert publisher.commands == [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    assert core.status().active_keys == []
