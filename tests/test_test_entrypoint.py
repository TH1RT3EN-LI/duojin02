import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("test_suites", ROOT / "tests/run_suites.py")
suites = importlib.util.module_from_spec(spec)
spec.loader.exec_module(suites)


def test_help_and_invalid_arguments_need_no_ros_or_docker():
    help_result = subprocess.run(
        [ROOT / "scripts/test.sh", "--help"], capture_output=True, text=True
    )
    assert help_result.returncode == 0
    for option in ("all", "quick", "navigation", "slam", "lidar", "--docker", "TEST_ROOT"):
        assert option in help_result.stdout
    invalid = subprocess.run([ROOT / "scripts/test.sh", "unknown"], capture_output=True, text=True)
    assert invalid.returncode != 0
    assert "invalid choice" in invalid.stderr


@pytest.mark.parametrize("value", ["0", "-1", "2.5", "invalid"])
def test_invalid_worker_settings_fail_preflight(monkeypatch, value):
    monkeypatch.setenv("BUILD_WORKERS", value)
    assert suites.main(["--preflight", "quick"]) == 1


def test_slam_source_is_checked_before_running_any_suite(monkeypatch, tmp_path):
    monkeypatch.setenv("TEST_ROOT", str(tmp_path / "not-created"))
    monkeypatch.setattr(suites, "prepared_slam_source", lambda: (_ for _ in ()).throw(
        RuntimeError("verified source missing")))
    assert suites.main(["--preflight", "all"]) == 1
    assert not (tmp_path / "not-created").exists()


@pytest.mark.parametrize("names", [[], ["unrelated"]])
def test_empty_or_incomplete_ctest_discovery_cannot_pass(monkeypatch, tmp_path, names):
    commands = []

    def run(command, **kwargs):
        commands.append(command)
        return json.dumps({"tests": [{"name": name} for name in names]}).encode()

    monkeypatch.setattr(suites, "run", run)
    with pytest.raises(RuntimeError, match="discovery incomplete"):
        suites.run_ctest(tmp_path, ["expected"])
    assert len(commands) == 1


def test_ctest_failures_propagate_and_empty_execution_is_an_error(monkeypatch, tmp_path):
    commands = []

    def run(command, **kwargs):
        commands.append(command)
        if "--show-only=json-v1" in command:
            return b'{"tests": [{"name": "expected"}]}'
        raise subprocess.CalledProcessError(8, command)

    monkeypatch.setattr(suites, "run", run)
    with pytest.raises(subprocess.CalledProcessError):
        suites.run_ctest(tmp_path, ["expected"])
    assert "--no-tests=error" in commands[-1]


def test_navigation_registers_each_probe_once(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", "221")
    probes = suites.navigation_probes(tmp_path / "install", tmp_path / "output")
    names = [name for name, _ in probes]
    assert len(names) == len(set(names)) == 8
    assert {"scan_fault_probe", "guard_tf_probe", "startup_parameter_probe"} <= set(names)
    assert "guard_association_probe" not in names  # Already registered in CTest.
    assert "guard_admission_probe" not in names
    for name, arguments in probes:
        if name in ("scan_fault_probe", "guard_tf_probe"):
            assert arguments[arguments.index("--domain-id") + 1] == "221"


def test_navigation_build_enables_tests_and_requested_cuda_architectures(monkeypatch, tmp_path):
    monkeypatch.setenv("BUILD_WORKERS", "2")
    monkeypatch.setenv("DUOJIN02_PYTHON", "/usr/bin/python3")
    monkeypatch.setenv("CUDA_ARCHITECTURES", "87;120")
    commands = []
    monkeypatch.setattr(suites, "run", lambda command: commands.append(command))
    monkeypatch.setattr(suites, "load_overlay", lambda path: None)
    suites.build_packages(tmp_path, ["duojin02_navigation"])
    assert "-DBUILD_TESTING=ON" in commands[0]
    assert "-DCMAKE_CUDA_ARCHITECTURES=87;120" in commands[0]
    assert "-DPython3_EXECUTABLE=/usr/bin/python3" in commands[0]
    assert "-DPYTHON_EXECUTABLE=/usr/bin/python3" in commands[0]
    assert commands[0][commands[0].index("--parallel-workers") + 1] == "2"


def test_native_cuda_default_uses_desktop_without_overriding_image_defaults(monkeypatch, tmp_path):
    monkeypatch.setenv("BUILD_WORKERS", "1")
    monkeypatch.setenv("DUOJIN02_PYTHON", "/usr/bin/python3")
    monkeypatch.delenv("CUDA_ARCHITECTURES", raising=False)
    commands = []
    monkeypatch.setattr(suites, "run", lambda command: commands.append(command))
    monkeypatch.setattr(suites, "load_overlay", lambda path: None)
    suites.build_packages(tmp_path, ["duojin02_navigation"])
    assert "-DCMAKE_CUDA_ARCHITECTURES=native" in commands[0]
    docker = suites.docker_command("navigation", tmp_path, "local:testing")
    assert not any(argument.startswith("CUDA_ARCHITECTURES=") for argument in docker)


def test_all_runs_each_suite_once_and_reports_failures(monkeypatch, tmp_path):
    monkeypatch.setenv("TEST_ROOT", str(tmp_path))
    monkeypatch.setattr(suites, "prepared_slam_source", lambda: tmp_path / "verified-slam")
    monkeypatch.setitem(
        sys.modules, "ros_isolation", SimpleNamespace(configure_isolation=lambda: 220)
    )
    called = []

    for name in suites.SUITES:
        def execute(directory, output, source, name=name):
            called.append(name)
            if name == "slam":
                raise subprocess.CalledProcessError(3, ["failing-probe"])
        monkeypatch.setattr(suites, name, execute)
    assert suites.main(["all"]) == 1
    assert called == ["quick", "navigation", "slam"]
    summary = json.loads(next(tmp_path.glob("run-*/summary.json")).read_text())
    assert not summary["success"]
    assert [entry["suite"] for entry in summary["passed"]] == ["quick", "navigation"]


def test_docker_builds_current_local_image_and_mounts_no_hardware(monkeypatch, tmp_path):
    monkeypatch.setenv("TEST_ROOT", str(tmp_path))
    monkeypatch.setenv("TEST_IMAGE", "local:test-current")
    monkeypatch.setenv("BUILD_JOBS", "3")
    monkeypatch.setenv("BUILD_WORKERS", "2")
    monkeypatch.delenv("PREPARED_SLAM_SOURCE", raising=False)
    commands = []
    monkeypatch.setattr(suites, "run", lambda command, **kwargs: commands.append((command, kwargs)))
    assert suites.main(["--docker", "navigation"]) == 0
    build, execution = commands
    assert build[0] == [ROOT / "scripts/build_image.sh"]
    assert build[1]["environment"]["IMAGE"] == "local:test-current"
    command = execution[0]
    assert command[-3:] == ["local:test-current", "./scripts/test.sh", "navigation"]
    assert f"{ROOT}:/ws:ro" in command and f"{tmp_path}:/check" in command
    assert command[command.index("--network") + 1] == "none"
    assert command[command.index("--entrypoint") + 1] == "/bin/bash"
    assert command[command.index("--cap-drop") + 1] == "ALL"
    assert "BUILD_JOBS=3" in command and "BUILD_WORKERS=2" in command
    assert not any(argument.startswith(("--device", "--privileged")) for argument in command)


def test_docker_default_image_matches_development_environment(monkeypatch, tmp_path):
    monkeypatch.setenv("TEST_ROOT", str(tmp_path))
    monkeypatch.delenv("TEST_IMAGE", raising=False)
    commands = []
    monkeypatch.setattr(suites, "run", lambda command, **kwargs: commands.append((command, kwargs)))
    suites.docker_suite("quick")
    assert commands[0][1]["environment"]["IMAGE"] == "duojin02:hardware-humble"
    assert commands[1][0][-3:] == ["duojin02:hardware-humble", "./scripts/test.sh", "quick"]
