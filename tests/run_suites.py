"""Implementation of the single local test entry point, scripts/test.sh."""

import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]
INTEGRATION = ROOT / "tests/integration"
SUITES = ("quick", "navigation", "slam", "lidar")
LAUNCHES = (
    "base", "mapping", "navigation", "navigation_stack", "online_navigation",
    "velocity_guard", "nav2", "localization", "slam", "offline_mapping", "save_map", "joy_teleop",
)


def parser():
    result = argparse.ArgumentParser(
        prog="./scripts/test.sh",
        description="Run local software checks using synthetic data and pseudo serial ports.",
        epilog=(
            "Environment: TEST_ROOT (default: build/tests), BUILD_JOBS (default: 2), "
            "BUILD_WORKERS (default: 1). Native SLAM tests require prepared patched source; "
            "PREPARED_SLAM_SOURCE may select it. Docker prepares its local development/test image "
            "before offline execution."
        ),
    )
    result.add_argument("suite", nargs="?", default="all", choices=("all", *SUITES))
    result.add_argument(
        "--docker", action="store_true",
        help="Use the local development/test Docker image (automatic without native Humble)",
    )
    result.add_argument("--preflight", action="store_true", help=argparse.SUPPRESS)
    return result


def positive_setting(name, default):
    value = os.environ.get(name, str(default))
    if not value.isdecimal() or int(value) < 1:
        raise ValueError(f"{name} must be a positive integer")
    return int(value)


def prepared_slam_source():
    sys.path.insert(0, str(INTEGRATION))
    from slam_test_source import resolve_source

    supplied = os.environ.get("PREPARED_SLAM_SOURCE")
    return resolve_source(ROOT, Path(supplied) if supplied else None)


def run(command, *, capture=False, environment=None, output_log=None):
    command = [str(part) for part in command]
    print(f"  $ {shlex.join(command)}", flush=True)
    if output_log is not None:
        try:
            with output_log.open("w") as stream:
                subprocess.run(command, cwd=ROOT, check=True, env=environment,
                               stdout=stream, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError:
            print("\n".join(output_log.read_text(errors="replace").splitlines()[-40:]),
                  file=sys.stderr)
            print(f"Full probe output: {output_log}", file=sys.stderr)
            raise
        print(f"  PASS; probe output: {output_log}", flush=True)
        return None
    try:
        result = subprocess.run(
            command, cwd=ROOT, check=True, capture_output=capture, env=environment
        )
    except subprocess.CalledProcessError as error:
        if capture:
            for stream, data in ((sys.stdout, error.stdout), (sys.stderr, error.stderr)):
                if data:
                    stream.write(os.fsdecode(data))
        raise
    return result.stdout if capture else None


def docker_command(suite, test_root, image):
    command = [
        "docker", "run", "--rm", "--init", "--entrypoint", "/bin/bash",
        "--network", "none", "--cap-drop", "ALL",
        "--user", f"{os.getuid()}:{os.getgid()}",
        "-v", f"{ROOT}:/ws:ro", "-v", f"{test_root}:/check", "-w", "/ws",
        "-e", "TEST_ROOT=/check", "-e", "ROS_LOG_DIR=/check/ros-logs",
        "-e", "PYTHONDONTWRITEBYTECODE=1", "-e", "PYTEST_ADDOPTS=-p no:cacheprovider",
        "-e", f"BUILD_JOBS={positive_setting('BUILD_JOBS', 2)}",
        "-e", f"BUILD_WORKERS={positive_setting('BUILD_WORKERS', 1)}",
    ]
    for setting in ("CUDA_ARCHITECTURES", "DUOJIN_ENABLE_CUDA"):
        if setting in os.environ:
            command.extend(["-e", f"{setting}={os.environ[setting]}"])
    if Path("/dev/nvidiactl").exists():
        command.extend(["--gpus", "all", "-e", "NVIDIA_DRIVER_CAPABILITIES=compute,utility"])
    supplied = os.environ.get("PREPARED_SLAM_SOURCE")
    if supplied:
        command.extend(["-v", f"{Path(supplied).expanduser().resolve()}:/prepared-slam:ro",
                        "-e", "PREPARED_SLAM_SOURCE=/prepared-slam"])
    return [*command, image, "./scripts/test.sh", suite]


def docker_suite(suite):
    test_root = Path(os.environ.get("TEST_ROOT", ROOT / "build/tests")).expanduser().resolve()
    test_root.mkdir(parents=True, exist_ok=True)
    image = os.environ.get("TEST_IMAGE", "duojin02:hardware-humble")
    print(f"Preparing current local Docker image: {image}", flush=True)
    run([ROOT / "scripts/build_image.sh"],
        environment={**os.environ, "IMAGE": image})
    run(docker_command(suite, test_root, image))


def load_overlay(path):
    # Source generated setup using positional arguments; do not interpolate shell code.
    output = run(
        ["bash", "-c", 'set -e; source "$1" >&2; env -0', "test-overlay", path],
        capture=True,
    )
    environment = {}
    for field in output.split(b"\0"):
        if b"=" in field:
            key, value = field.split(b"=", 1)
            environment[os.fsdecode(key)] = os.fsdecode(value)
    if not environment:
        raise RuntimeError(f"Overlay returned no environment: {path}")
    os.environ.clear()
    os.environ.update(environment)


def run_ctest(directory, required):
    discovered = json.loads(
        run(["ctest", "--test-dir", directory, "--show-only=json-v1"], capture=True)
    )
    names = {test["name"] for test in discovered.get("tests", [])}
    if not names or not set(required) <= names:
        raise RuntimeError(
            f"CTest discovery incomplete in {directory}: "
            f"missing {sorted(set(required) - names)}, found {len(names)} tests"
        )
    print(f"  CTest discovered {len(names)} tests", flush=True)
    run(["ctest", "--test-dir", directory, "--output-on-failure", "--no-tests=error"])


def build_packages(directory, packages):
    packages = list(packages)
    if "duojin02_navigation" in packages:
        packages = ["duojin02_mppi_controller", "duojin02_amcl_cuda", *packages]
    python = os.environ["DUOJIN02_PYTHON"]
    options = [
        "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=ON",
        f"-DPython3_EXECUTABLE={python}", f"-DPYTHON_EXECUTABLE={python}",
    ]
    if "duojin02_navigation" in packages:
        options.append(f"-DCMAKE_CUDA_ARCHITECTURES={os.environ.get('CUDA_ARCHITECTURES', 'native')}")
        if "DUOJIN_ENABLE_CUDA" in os.environ:
            options.append(f"-DDUOJIN_ENABLE_CUDA={os.environ['DUOJIN_ENABLE_CUDA']}")
    run([
        "colcon", "--log-base", directory / "log", "build", "--base-paths",
        *(ROOT / "src" / package for package in packages),
        "--build-base", directory / "build", "--install-base", directory / "install",
        "--merge-install", "--parallel-workers", os.environ["BUILD_WORKERS"],
        "--cmake-force-configure", "--cmake-args", *options,
    ])
    load_overlay(directory / "install/setup.bash")


def probe(name, *arguments):
    output = Path(os.environ["TEST_ARTIFACT_ROOT"]) / f"{name}.log"
    run([sys.executable, INTEGRATION / f"{name}.py", *arguments], output_log=output)


def navigation_probes(install, output):
    binaries = [
        "--guard-binary", install / "lib/duojin02_navigation/velocity_guard",
        "--driver-binary", install / "lib/duojin02_base_driver/duojin02_base_driver_node",
    ]
    guard = binaries[:2]
    safety = ROOT / "src/duojin02_bringup/config/safety.yaml"
    navigation = ROOT / "src/duojin02_bringup/config/nav2.yaml"
    return (
        ("guard_map_validation_probe", [*guard, "--params-file", safety,
            "--json-out", output / "grid-validation.json", "--log-dir", output / "grid-logs"]),
        ("localization_monitor_probe", [*binaries, "--output-dir", output / "localization"]),
        ("navigation_command_timeout_probe", [*binaries, "--params", navigation, "--candidate",
            "--output-dir", output / "command-timeout"]),
        ("manual_arbitration_probe", [*binaries, "--output-dir", output / "manual-arbitration"]),
        ("navigation_recovery_probe", ["--params-file", navigation, "--tree-directory",
            ROOT / "src/duojin02_bringup/behavior_trees", "--output-dir", output / "recovery"]),
        ("scan_fault_probe", [*guard, "--params-file", safety, "--domain-id",
            os.environ["ROS_DOMAIN_ID"], "--json-out", output / "scan-fault.json",
            "--log-dir", output / "scan-fault-logs"]),
        ("guard_tf_probe", [*guard, "--params-file", safety, "--domain-id",
            os.environ["ROS_DOMAIN_ID"], "--json-out", output / "guard-tf.json",
            "--log-dir", output / "guard-tf-logs"]),
        ("startup_parameter_probe", [*binaries, "--params-file", safety,
            "--output-dir", output / "startup-parameters"]),
    )


def quick(directory, output, slam_source=None):
    run([sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", "tests",
         "src/duojin02_teleop/test/test_keyboard_teleop_core.py"])
    for entry in LAUNCHES:
        run(
            ["ros2", "launch", "duojin02_bringup", f"{entry}.launch.py", "--show-args"],
            capture=True,
        )
    run(["ros2", "launch", "lslidar_driver", "lslidar_x10_launch.py", "--show-args"], capture=True)


def navigation(directory, output, slam_source=None):
    build_packages(directory, ["duojin02_navigation", "duojin02_base_driver",
                               "duojin02_bringup", "duojin02_teleop"])
    run_ctest(directory / "build/duojin02_navigation", [
        "recovery_bt_checks", "gpu_geometry_checks", "guard_association_probe",
        "guard_admission_probe", "scan_validity_checks",
    ])
    run_ctest(directory / "build/duojin02_base_driver", ["feedback_pipeline_checks"])
    for name, arguments in navigation_probes(directory / "install", output):
        probe(name, *arguments)


def slam(directory, output, slam_source=None):
    python = os.environ["DUOJIN02_PYTHON"]
    run(["cmake", "-S", INTEGRATION, "-B", directory / "build",
         f"-DPREPARED_SLAM_SOURCE={slam_source}",
         f"-DPython3_EXECUTABLE={python}", f"-DPYTHON_EXECUTABLE={python}"])
    run(["cmake", "--build", directory / "build", "--parallel", os.environ["BUILD_JOBS"]])
    for name in ("scan_gate_probe", "ceres_anchor_probe", "map_saver_completion_probe"):
        run([sys.executable, INTEGRATION / "ros_isolation.py", directory / "build" / name])
    for name in ("base_feedback_probe", "map_save_service_probe", "recording_probe"):
        probe(name)
    probe("slam_replay_probe", output / "replay")


def lidar(directory, output, slam_source=None):
    build_packages(directory, ["lslidar_driver"])
    run_ctest(directory / "build/lslidar_driver", ["test_x10_driver"])
    probe("lidar_output_probe")


def main(arguments=None):
    args = parser().parse_args(arguments)
    try:
        positive_setting("BUILD_JOBS", 2)
        positive_setting("BUILD_WORKERS", 1)
        if args.docker:
            if not args.preflight:
                docker_suite(args.suite)
            return 0
        selected = SUITES if args.suite == "all" else (args.suite,)
        slam_source = prepared_slam_source() if "slam" in selected else None
        if args.preflight:
            return 0

        sys.path.insert(0, str(INTEGRATION))
        from ros_isolation import configure_isolation

        domain = configure_isolation()
        test_root = Path(os.environ.get("TEST_ROOT", ROOT / "build/tests")).expanduser().resolve()
        test_root.mkdir(parents=True, exist_ok=True)
        artifacts = Path(tempfile.mkdtemp(prefix="run-", dir=test_root))
        os.environ.update(
            TEST_ROOT=str(test_root), TEST_ARTIFACT_ROOT=str(artifacts),
            ROS_LOG_DIR=str(artifacts / "ros-logs"), PYTHONDONTWRITEBYTECODE="1",
        )
        print(
            f"Test suite: {args.suite}; isolated ROS domain: {domain}; output: {artifacts}",
            flush=True,
        )
        completed = []
        try:
            for name in selected:
                print(f"\n[{name}] Starting", flush=True)
                output = artifacts / name
                output.mkdir()
                began = time.monotonic()
                globals()[name](test_root / name, output, slam_source)
                completed.append({"suite": name, "seconds": round(time.monotonic() - began, 2)})
                print(f"[{name}] PASS", flush=True)
        finally:
            (artifacts / "summary.json").write_text(json.dumps({
                "requested": list(selected), "passed": completed,
                "success": len(completed) == len(selected),
            }, indent=2) + "\n")
        print(f"\nPASS: {len(completed)} suite(s); results: {artifacts}", flush=True)
        return 0
    except (ValueError, RuntimeError, OSError, subprocess.CalledProcessError) as error:
        print(f"Test failure: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
