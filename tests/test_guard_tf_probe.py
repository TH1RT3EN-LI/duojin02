import ast
from pathlib import Path
import time
from types import SimpleNamespace

import pytest


PATH = Path(__file__).resolve().parent / "integration/guard_tf_probe.py"


def require(condition, message, **evidence):
    assert condition, (message, evidence)


def barrier_function():
    tree = ast.parse(PATH.read_text())
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef)
                    and node.name == "pending_fault_case")
    namespace = {
        "time": time,
        "require": require,
        "stamp_ns": lambda stamp: stamp.nanosec,
        "zero": lambda command: command["linear_x"] == 0,
        "clear_command": lambda command: command["linear_x"] == 0.3,
    }
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(PATH), "exec"), namespace)
    return namespace["pending_fault_case"]


class Probe:
    def __init__(self, defect=None):
        self.defect = defect
        self.events = []
        self.statuses = [(0, {"reason": "clear", "tf_ready_scans": 1})]
        self.outputs = [(0, {"linear_x": 0.3})]
        self.clouds = []
        self.next_stamp = 0
        self.scan_pub = SimpleNamespace(publish=self.publish)

    def scan(self):
        self.next_stamp += 1
        return SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(nanosec=self.next_stamp), frame_id=""), fault=False)

    def fault_scan(self, kind):
        scan = self.scan()
        scan.fault = True
        return scan

    def publish(self, scan):
        stamp = scan.header.stamp.nanosec
        self.events.append(f"scan_{stamp}")
        if scan.fault:
            self.statuses.append((0, {"reason": "scan_invalid", "tf_ready_scans": 1}))
            self.outputs.append((0, {"linear_x": 0}))
        elif stamp == 1 and "fault_ack" in self.events:
            if self.defect == "old_admitted":
                self.statuses[-1][1]["tf_ready_scans"] = 2
            elif self.defect == "old_restored_motion":
                self.statuses[-1][1]["reason"] = "clear"
                self.outputs.append((0, {"linear_x": 0.3}))
            elif self.defect == "old_polluted_memory":
                self.clouds.append((0, scan))
        elif stamp == 3:
            count = 1 if self.defect == "fresh_not_counted" else 2
            self.statuses.append((0, {"reason": "clear", "tf_ready_scans": count}))
            self.outputs.append((0, {"linear_x": 0.3}))

    def response(self, since, status_predicate, output_predicate):
        require(status_predicate(self.statuses[-1][1]), "Expected status missing")
        require(output_predicate(self.outputs[-1][1]), "Expected command missing")
        name = "fault_ack" if self.statuses[-1][1]["reason"] == "scan_invalid" else "fresh_ack"
        self.events.append(name)
        return {"command_latency_ms": 1}

    def pump(self, duration):
        self.events.append(("observe", duration))


def test_old_scan_tf_and_replay_follow_observed_fault_without_sleeping_for_admission():
    probe = Probe()
    recovery, evidence = barrier_function()(
        probe, lambda scan: probe.events.append(f"tf_{scan.header.stamp.nanosec}")
    )
    assert probe.events == [
        "scan_1", "scan_2", "fault_ack", "tf_1", "scan_1", ("observe", 0.070),
        "tf_3", "scan_3", "fresh_ack",
    ]
    assert evidence["tf_ready_after_old_replay"] == 1
    assert evidence["tf_ready_after_fresh_recovery"] == 2
    assert recovery.header.stamp.nanosec > evidence["fault_stamp_ns"]


@pytest.mark.parametrize("defect", [
    "old_admitted", "old_restored_motion", "old_polluted_memory", "fresh_not_counted",
])
def test_barrier_probe_rejects_motion_memory_and_admission_regressions(defect):
    with pytest.raises(AssertionError):
        barrier_function()(Probe(defect), lambda scan: None)
