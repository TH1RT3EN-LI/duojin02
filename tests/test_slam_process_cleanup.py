import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/duojin02_slam_tools"))
from duojin02_slam_tools.offline import spawn_owned_process, stop_process


pytestmark = pytest.mark.skipif(
    not hasattr(os, "pidfd_open"), reason="Linux pidfd ownership contract")


@pytest.fixture
def subreaper():
    # Reap the real orphan fixture ourselves instead of leaving a zombie to the
    # container's PID1. This changes no production process-management policy.
    libc = ctypes.CDLL(None, use_errno=True)
    original = ctypes.c_int()
    assert libc.prctl(37, ctypes.byref(original), 0, 0, 0) == 0
    assert libc.prctl(36, 1, 0, 0, 0) == 0
    try:
        yield
    finally:
        assert libc.prctl(36, original.value, 0, 0, 0) == 0


def wait_file(path, timeout=5):
    deadline = time.monotonic() + timeout
    while True:
        try:
            return json.loads(path.read_text())
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Fixture did not create {path}")
        time.sleep(0.01)


def test_reaped_leader_does_not_hide_a_live_owned_child(tmp_path, subreaper):
    ready = tmp_path / "child.json"
    code = r'''
import json,os,signal,sys,time
child=os.fork()
if child:
    os._exit(0)
signal.signal(signal.SIGTERM,lambda *_:os._exit(0))
with open(sys.argv[1],"w") as f:json.dump({"pid":os.getpid()},f)
while True:time.sleep(.05)
'''
    leader = spawn_owned_process([sys.executable, "-c", code, str(ready)])
    child = None
    try:
        assert leader.wait(timeout=5) == 0
        child = wait_file(ready)["pid"]
        assert Path(f"/proc/{child}/stat").exists()
        stop_process(leader, launch=True)
        assert os.waitpid(child, 0)[0] == child
        child = None
        # Repeated cleanup of a now-empty owned session is safe.
        stop_process(leader, launch=True)
    finally:
        stop_process(leader, launch=True)
        if child is not None:
            os.waitpid(child, 0)


def test_normal_launch_forwards_one_interrupt_without_duplicate_child_int(tmp_path):
    ready = tmp_path / "child.json"
    events = tmp_path / "signals.txt"
    code = r'''
import json,os,signal,sys,time
def record(value):
    with open(sys.argv[2],"a") as f:f.write(value+"\n")
child=[None]
read_end,write_end=os.pipe()
def wrapper_int(*_):
    record("wrapper_INT")
    os.kill(child[0],signal.SIGINT)
signal.signal(signal.SIGINT,wrapper_int)
pid=os.fork()
if not pid:
    os.close(write_end)
    os.read(read_end,1)
    os.close(read_end)
    stop=[None]
    def child_int(*_):
        record("child_INT")
        stop[0]=time.monotonic()+.15
    signal.signal(signal.SIGINT,child_int)
    with open(sys.argv[1],"w") as f:json.dump({"pid":os.getpid()},f)
    while stop[0] is None or time.monotonic()<stop[0]:time.sleep(.005)
    os._exit(0)
child[0]=pid
os.close(read_end)
os.write(write_end,b"x")
os.close(write_end)
while True:
    try:
        if os.waitpid(pid,0)[0]==pid:break
    except InterruptedError:pass
'''
    leader = spawn_owned_process([sys.executable, "-c", code, str(ready), str(events)])
    try:
        wait_file(ready)
        stop_process(leader, launch=True)
        assert leader.returncode == 0
        assert events.read_text().splitlines() == ["wrapper_INT", "child_INT"]
    finally:
        stop_process(leader, launch=True)


@pytest.mark.parametrize("tamper", ["token", "leader_start_ticks"])
def test_unverified_identity_is_refused_without_signalling_a_live_process(tamper):
    leader = spawn_owned_process([sys.executable, "-c", "import time;time.sleep(30)"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    owner = leader._slam_process_owner
    original = owner[tamper]
    try:
        owner[tamper] = "unrelated-owner" if tamper == "token" else original + 1
        with pytest.raises(RuntimeError, match="[Uu]nverified|identity changed"):
            stop_process(leader, launch=True)
        assert leader.poll() is None
    finally:
        owner[tamper] = original
        stop_process(leader, launch=True)


def test_live_owned_process_with_unreadable_environment_is_not_signalled(monkeypatch):
    from duojin02_slam_tools import offline

    leader = spawn_owned_process([sys.executable, "-c", "import time;time.sleep(30)"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    read_token = offline._process_token

    def unreadable_token(pid):
        if pid == leader.pid:
            raise PermissionError("fixture denies access to a live process environment")
        return read_token(pid)

    try:
        with monkeypatch.context() as patch:
            patch.setattr(offline, "_process_token", unreadable_token)
            with pytest.raises(RuntimeError, match="Cannot verify member"):
                stop_process(leader, launch=True)
            assert leader.poll() is None
    finally:
        stop_process(leader, launch=True)
