import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]


def installer_fixture(tmp_path, *, uid="1000", version="22.04", ros=True, initialized=False):
    dependency_root = tmp_path / "dependencies"
    scripts = dependency_root / "scripts"
    scripts.mkdir(parents=True)
    (dependency_root / "src").mkdir()
    (dependency_root / "requirements-build.txt").write_bytes(
        (ROOT / "requirements-build.txt").read_bytes()
    )
    binary = tmp_path / "bin"
    binary.mkdir()
    calls = tmp_path / "calls.jsonl"
    os_release = tmp_path / "os-release"
    os_release.write_text(f'ID=ubuntu\nVERSION_ID="{version}"\n')
    ros_setup = tmp_path / "ros-setup.bash"
    if ros:
        ros_setup.write_text("export INSTALLER_ROS_SETUP_LOADED=1\n")
    rosdep_list = tmp_path / "rosdep.list"
    if initialized:
        rosdep_list.touch()

    # Redirect operating-system files and the fixed system Python into the fixture;
    # the shell still executes the installer's real control flow and command options.
    contents = (ROOT / "scripts/install_dependencies.sh").read_text()
    for original, replacement in (
        ("/etc/os-release", os_release),
        ("/opt/ros/humble/setup.bash", ros_setup),
        ("/etc/ros/rosdep/sources.list.d/20-default.list", rosdep_list),
        ("/usr/bin/python3", binary / "python3"),
    ):
        contents = contents.replace(original, shlex.quote(str(replacement)))
    script = scripts / "install_dependencies.sh"
    script.write_text(contents)
    stub = (
        f"#!{sys.executable}\n"
        + """import json,os,sys
from pathlib import Path
program=Path(sys.argv[0]).name
args=sys.argv[1:]
with Path(os.environ['DEPENDENCY_TEST_CALLS']).open('a') as stream:
    stream.write(json.dumps({'program':program,'args':args,
                             'ros_loaded':os.environ.get('INSTALLER_ROS_SETUP_LOADED')})+'\\n')
if program=='id':
    print(os.environ['DEPENDENCY_TEST_UID'])
elif program=='sudo':
    os.execvp(args[0],args)
elif program=='rosdep' and args[0]=='install':
    raise SystemExit(int(os.environ.get('DEPENDENCY_TEST_ROSDEP_FAILURE','0')))
"""
    )
    for name in ("id", "sudo", "apt-get", "rosdep", "python3"):
        command = binary / name
        command.write_text(stub)
        command.chmod(0o755)
    environment = {
        **os.environ,
        "PATH": str(binary) + os.pathsep + os.environ["PATH"],
        "DEPENDENCY_TEST_CALLS": str(calls),
        "DEPENDENCY_TEST_UID": uid,
    }
    environment.pop("INSTALLER_ROS_SETUP_LOADED", None)
    return script, dependency_root, calls, environment


def run_installer(script, calls, environment):
    result = subprocess.run(
        ["bash", str(script)],
        cwd=script.parents[2],
        env=environment,
        capture_output=True,
        text=True,
        timeout=5,
    )
    commands = (
        [json.loads(line) for line in calls.read_text().splitlines()] if calls.exists() else []
    )
    return result, commands


@pytest.mark.parametrize("uid", ["0", "1000"])
def test_shared_installer_uses_manifest_root_and_correct_install_scope(tmp_path, uid):
    script, dependency_root, calls, environment = installer_fixture(tmp_path, uid=uid)
    result, commands = run_installer(script, calls, environment)
    assert result.returncode == 0, result.stderr
    actual = [command for command in commands if command["program"] not in ("id", "sudo")]
    assert [command["program"] for command in actual] == [
        "apt-get", "apt-get", "rosdep", "rosdep", "rosdep", "python3", "python3"
    ]
    assert actual[0]["args"] == ["update", "-o", "APT::Update::Error-Mode=any"]
    assert actual[1]["args"][:3] == ["install", "-y", "--no-install-recommends"]
    assert set(actual[1]["args"][3:]) == {
        "build-essential", "git", "curl", "ca-certificates", "pkg-config", "python3-pip",
        "python3-colcon-common-extensions", "python3-rosdep", "python3-pytest",
    }
    assert actual[2]["args"] == ["init"]
    assert actual[3]["args"] == ["update", "--rosdistro", "humble"]
    assert actual[4]["args"] == [
        "install", "--from-paths", str(dependency_root / "src"),
        "--ignore-src", "--rosdistro", "humble", "-y",
    ]
    assert all(command["ros_loaded"] == "1" for command in actual[2:])
    assert actual[5]["args"] == [str(dependency_root / "scripts/fix_humble_cmake.py")]
    assert actual[6]["args"] == [
        "-m", "pip", "install", "--no-cache-dir",
        *(["--user"] if uid != "0" else []),
        "-r", str(dependency_root / "requirements-build.txt"),
    ]
    privileged = [command["args"][0] for command in commands if command["program"] == "sudo"]
    assert privileged == (
        ["apt-get", "apt-get", "rosdep", str(tmp_path / "bin/python3")] if uid != "0" else []
    )


def test_existing_rosdep_initialization_is_preserved(tmp_path):
    script, _, calls, environment = installer_fixture(tmp_path, initialized=True)
    result, commands = run_installer(script, calls, environment)
    assert result.returncode == 0, result.stderr
    assert not any(command["args"] == ["init"] for command in commands)


@pytest.mark.parametrize("options", [{"version": "26.04"}, {"ros": False}])
def test_unsupported_environment_is_rejected_before_installation(tmp_path, options):
    script, _, calls, environment = installer_fixture(tmp_path, **options)
    result, commands = run_installer(script, calls, environment)
    assert result.returncode != 0
    assert "Humble" in result.stderr
    assert not commands


def test_rosdep_failure_stops_before_system_fix_or_pip(tmp_path):
    script, _, calls, environment = installer_fixture(tmp_path)
    environment["DEPENDENCY_TEST_ROSDEP_FAILURE"] = "7"
    result, commands = run_installer(script, calls, environment)
    assert result.returncode == 7
    assert not any(command["program"] == "python3" for command in commands)
