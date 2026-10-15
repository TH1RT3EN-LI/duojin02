import json
import os
from pathlib import Path
import platform
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "scripts/environment.sh"


@pytest.fixture
def installation(tmp_path):
    workspace = tmp_path / "workspace"
    image = tmp_path / "image"
    tools = tmp_path / "tools"
    modules = tmp_path / "modules"
    for directory in (workspace, image, tools, modules):
        directory.mkdir()
    (tools / "cmake").write_text("#!/bin/sh\nexit 0\n")
    (tools / "cmake").chmod(0o755)
    (modules / "cmake.py").write_text(f"CMAKE_BIN_DIR = {str(tools)!r}\n")
    ros = tmp_path / "ros-setup.bash"
    ros.write_text('printf "ros\\n" >> "$ENV_TEST_TRACE"\n')
    trace = tmp_path / "loaded.log"
    environment = dict(os.environ, PYTHONPATH=str(modules), ENV_TEST_TRACE=str(trace))
    for variable in (
        "ROS_DOMAIN_ID",
        "RMW_IMPLEMENTATION",
        "DUOJIN02_WORKSPACE_ROOT",
        "DUOJIN02_PYTHON",
        "BUILD_JOBS",
        "BUILD_WORKERS",
        "CMAKE_BUILD_PARALLEL_LEVEL",
        "MAKEFLAGS",
    ):
        environment.pop(variable, None)
    return workspace, image, ros, tools, trace, environment


def overlay(root, name, label, architecture=platform.machine(), commands=""):
    directory = root / "install" / name
    directory.mkdir(parents=True)
    (directory / "setup.bash").write_text(
        f'printf "{label}\\n" >> "$ENV_TEST_TRACE"\n{commands}'
    )
    if architecture is not None:
        (directory / ".target_architecture").write_text(architecture + "\n")
    return directory


def run_environment(installation, mode="runtime", overrides=None):
    workspace, image, ros, _, _, environment = installation
    environment = {**environment, **(overrides or {})}
    script = """
source "$1"
duojin02_setup_environment "$2" "$3" "$4" "$5"
environment_status=$?
/usr/bin/python3 -c 'import json, os; print(json.dumps(dict(os.environ)))'
exit "$environment_status"
"""
    result = subprocess.run(
        [
            "bash",
            "-c",
            script,
            "environment-test",
            str(HELPER),
            mode,
            str(workspace),
            str(image),
            str(ros),
        ],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
    )
    return result, json.loads(result.stdout)


def test_workspace_overlays_win_and_preserve_user_runtime_settings(installation):
    workspace, image, ros, _, trace, _ = installation
    for root, label in ((workspace, "workspace"), (image, "image")):
        for name in ("slam_backend", "hardware"):
            overlay(root, name, f"{label}-{name}")
    ros.write_text(
        ros.read_text()
        + "export ROS_DOMAIN_ID=99 RMW_IMPLEMENTATION=other DUOJIN02_WORKSPACE_ROOT=/other\n"
    )
    settings = {
        "ROS_DOMAIN_ID": "73",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "DUOJIN02_WORKSPACE_ROOT": "/recorded-data",
    }
    result, environment = run_environment(installation, overrides=settings)
    assert result.returncode == 0, result.stderr
    assert trace.read_text().splitlines() == ["ros", "workspace-slam_backend", "workspace-hardware"]
    assert {key: environment[key] for key in settings} == settings


def test_image_overlays_supply_an_unbuilt_mounted_workspace(installation):
    workspace, image, _, _, trace, _ = installation
    for name in ("slam_backend", "hardware"):
        overlay(image, name, f"image-{name}")
    result, environment = run_environment(installation)
    assert result.returncode == 0, result.stderr
    assert trace.read_text().splitlines() == ["ros", "image-slam_backend", "image-hardware"]
    assert environment["ROS_DOMAIN_ID"] == "20"
    assert environment["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert environment["DUOJIN02_WORKSPACE_ROOT"] == str(workspace)


@pytest.mark.parametrize("name", ["slam_backend", "hardware"])
@pytest.mark.parametrize("architecture", ["wrong-architecture", None])
def test_invalid_workspace_architecture_is_rejected_before_loading_any_setup(
    installation, name, architecture
):
    workspace, image, _, _, trace, _ = installation
    for selected in ("slam_backend", "hardware"):
        overlay(image, selected, f"image-{selected}")
        overlay(
            workspace,
            selected,
            f"workspace-{selected}",
            architecture=architecture if selected == name else platform.machine(),
        )
    result, _ = run_environment(installation)
    assert result.returncode != 0
    assert "architecture" in result.stderr
    assert not trace.exists()


@pytest.mark.parametrize("name", ["slam_backend", "hardware"])
@pytest.mark.parametrize("has_setup", [True, False])
def test_partial_workspace_never_mixes_with_complete_image(installation, name, has_setup):
    workspace, image, _, _, trace, _ = installation
    for selected in ("slam_backend", "hardware"):
        overlay(image, selected, f"image-{selected}")
    if has_setup:
        overlay(workspace, name, f"workspace-{name}")
    else:
        (workspace / "install" / name).mkdir(parents=True)
    result, _ = run_environment(installation)
    assert result.returncode != 0
    assert "has not been built" in result.stderr
    assert not trace.exists()


def test_missing_runtime_installation_returns_failure_without_sourcing_ros(installation):
    *_, trace, _ = installation
    result, _ = run_environment(installation)
    assert result.returncode != 0
    assert "has not been built" in result.stderr
    assert not trace.exists()


def test_overlay_source_failure_preserves_status_and_stops_the_chain(installation):
    workspace, _, _, _, trace, _ = installation
    overlay(workspace, "slam_backend", "slam", commands="return 37\n")
    overlay(workspace, "hardware", "hardware")
    result, _ = run_environment(installation)
    assert result.returncode == 37
    assert trace.read_text().splitlines() == ["ros", "slam"]


def test_build_mode_ignores_old_overlays_and_selects_system_python_and_shared_limits(installation):
    workspace, image, _, tools, trace, environment = installation
    for root in (workspace, image):
        for name in ("slam_backend", "hardware"):
            overlay(
                root,
                name,
                "must-not-load",
                architecture="wrong-architecture",
                commands="return 81\n",
            )
    fake_python = tools / "python3"
    fake_python.write_text("#!/bin/sh\nexit 91\n")
    fake_python.chmod(0o755)
    result, selected = run_environment(
        installation,
        mode="build",
        overrides={
            "PATH": f"{tools}:{environment['PATH']}",
            "DUOJIN02_PYTHON": str(fake_python),
            "BUILD_JOBS": "3",
            "BUILD_WORKERS": "2",
        },
    )
    assert result.returncode == 0, result.stderr
    assert trace.read_text().splitlines() == ["ros"]
    assert selected["DUOJIN02_PYTHON"] == "/usr/bin/python3"
    assert selected["BUILD_JOBS"] == selected["CMAKE_BUILD_PARALLEL_LEVEL"] == "3"
    assert selected["BUILD_WORKERS"] == "2" and selected["MAKEFLAGS"] == "-j3"
    assert selected["PATH"].split(":")[0] == str(tools)


def test_build_mode_defaults_work_without_any_installation(installation):
    result, environment = run_environment(installation, mode="build")
    assert result.returncode == 0, result.stderr
    assert environment["BUILD_JOBS"] == environment["CMAKE_BUILD_PARALLEL_LEVEL"] == "2"
    assert environment["BUILD_WORKERS"] == "1" and environment["MAKEFLAGS"] == "-j2"


@pytest.mark.parametrize("variable,value", [("BUILD_JOBS", "0"), ("BUILD_WORKERS", "-1")])
def test_invalid_build_limits_are_rejected(installation, variable, value):
    result, _ = run_environment(installation, mode="build", overrides={variable: value})
    assert result.returncode != 0
    assert "positive integers" in result.stderr


def test_source_wrapper_returns_failure_without_exiting_the_calling_shell(tmp_path):
    (tmp_path / "scripts").mkdir()
    shutil.copyfile(ROOT / "source.sh", tmp_path / "source.sh")
    (tmp_path / "scripts/environment.sh").write_text(
        'duojin02_setup_environment() { printf "%s\\n" "$1" "$2"; return 37; }\n'
    )
    result = subprocess.run(
        [
            "bash",
            "-c",
            'source "$1"; status=$?; printf "survived:%s\\n" "$status"',
            "test",
            str(tmp_path / "source.sh"),
        ],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0
    assert result.stdout.splitlines() == ["runtime", str(tmp_path), "survived:37"]


def test_enter_uses_the_same_entrypoint_as_a_new_container(tmp_path):
    docker = tmp_path / "docker"
    docker.write_text(
        '#!/bin/bash\nif [[ "$1" == ps ]]; then echo environment-test; '
        'else printf "%s\\n" "$@"; fi\n'
    )
    docker.chmod(0o755)
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/enter.sh")],
        env={**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}", "NAME": "environment-test"},
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines()[-3:] == [
        "environment-test",
        "/usr/local/bin/duojin02-entrypoint",
        "bash",
    ]
