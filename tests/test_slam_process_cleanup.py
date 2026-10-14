from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/duojin02_slam_tools"))
from duojin02_slam_tools import offline


@pytest.mark.parametrize("vanished_signal", [signal.SIGINT, signal.SIGTERM, signal.SIGKILL])
def test_process_group_exit_during_each_shutdown_stage_is_harmless(monkeypatch, vanished_signal):
    signals = []

    def kill(pid, value):
        signals.append(value)
        if value == vanished_signal:
            raise ProcessLookupError("child exited concurrently")

    def wait(timeout):
        raise subprocess.TimeoutExpired("probe", timeout)

    monkeypatch.setattr(offline.os, "killpg", kill)
    process = SimpleNamespace(pid=42, poll=lambda: None, wait=wait)
    offline.stop_process(process)
    assert signals[-1] == vanished_signal


def test_launch_wrapper_exit_during_signal_is_harmless():
    def vanished(value):
        raise ProcessLookupError("launch exited concurrently")

    process = SimpleNamespace(poll=lambda: None, send_signal=vanished)
    offline.stop_process(process, launch=True)


def test_normal_launch_shutdown_signals_only_wrapper(monkeypatch):
    signals, waits = [], []
    process = SimpleNamespace(
        poll=lambda: None, send_signal=signals.append, wait=lambda timeout: waits.append(timeout)
    )

    def duplicate(*args):
        pytest.fail("ros2 launch must not receive duplicate group SIGINT")

    monkeypatch.setattr(offline.os, "killpg", duplicate)
    offline.stop_process(process, launch=True)
    assert signals == [signal.SIGINT] and waits == [10]


def test_stalled_child_escalation_stays_bounded(monkeypatch):
    signals, waits = [], []
    monkeypatch.setattr(offline.os, "killpg", lambda pid, value: signals.append(value))

    def wait(timeout):
        waits.append(timeout)
        if len(waits) < 3:
            raise subprocess.TimeoutExpired("probe", timeout)

    process = SimpleNamespace(pid=42, poll=lambda: None, wait=wait)
    offline.stop_process(process)
    assert signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert waits == [10, 5, 5]


def test_already_exited_child_needs_no_signal(monkeypatch):
    def unexpected(*args):
        pytest.fail("completed child should not be signalled")

    monkeypatch.setattr(offline.os, "killpg", unexpected)
    offline.stop_process(SimpleNamespace(poll=lambda: 0))
    offline.stop_process(None)


def test_disposable_metadata_child_cannot_ignore_timeout_termination():
    began = time.monotonic()
    completed = subprocess.run(
        [
            "timeout",
            "--kill-after=1s",
            "0.2s",
            sys.executable,
            "-c",
            "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(10)",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=3,
    )
    assert completed.returncode == -signal.SIGKILL or completed.returncode == 128 + signal.SIGKILL
    assert time.monotonic() - began < 2.5
