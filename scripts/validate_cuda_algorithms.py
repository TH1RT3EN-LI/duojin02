#!/usr/bin/env python3
"""Build and validate the installed local MPPI/AMCL CUDA implementation in Docker."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ("duojin02_mppi_controller", "duojin02_amcl_cuda", "duojin02_navigation")


def manifest():
    paths = [p for folder in ("src", "scripts", "tests") for p in (ROOT / folder).rglob("*")
             if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(paths)}
    return {"sha256": hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest(),
            "files": hashes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default="duojin02:hardware-humble")
    parser.add_argument("--cpu-image", help="Also validate a toolkit-free CPU install using this image")
    parser.add_argument("--architecture", default="120")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--frames", type=int, default=60)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.jobs < 1 or args.frames < 10:
        parser.error("jobs must be positive and frames must be >= 10")
    epoch = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
    output = (args.output or ROOT / "output/cuda-local-v3" / epoch).resolve()
    relative = output.relative_to(ROOT)
    output.mkdir(parents=True, exist_ok=False)
    inside_output = "/workspace/" + str(relative)
    name = "duojin02-algorithms-" + uuid.uuid4().hex[:10]
    record = {"schema_version": 1, "status": "running", "started_utc": datetime.now(timezone.utc).isoformat(),
              "source_before": manifest(), "commands": [], "tests": {},
              "scope": "local GPU component parity and actual installed AMCL synthetic replay; no hardware qualification",
              "image": args.image, "architecture": args.architecture}

    def save():
        (output / "validation.json").write_text(json.dumps(record, indent=2) + "\n")

    def run(stage, command, expected=0):
        print(f"[{stage}] {shlex.join(command)}", flush=True)
        start = time.monotonic()
        with (output / f"{stage}.log").open("w") as stream:
            result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                    timeout=900 if "build" in stage else 240)
        record["commands"].append({"stage": stage, "argv": command, "returncode": result.returncode,
                                   "expected": expected, "seconds": time.monotonic() - start})
        save()
        if result.returncode != expected:
            print((output / f"{stage}.log").read_text(errors="replace")[-8000:])
            raise RuntimeError(f"{stage} returned {result.returncode}, expected {expected}")

    def inside(stage, command, expected=0):
        run(stage, ["docker", "exec", name, "bash", "-c", command], expected)

    setup = "source /workspace/scripts/source_cuda_local.sh && "

    def artifact_manifest(variant):
        return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted((ROOT / f"install/{variant}-local-v3").rglob("*")) if p.is_file()}

    def ctest(package):
        stage = package + "-tests"
        inside(stage, setup + f"ctest --test-dir build/cuda-local-v3/{package} "
               f"--no-tests=error --output-on-failure --output-junit {shlex.quote(inside_output + '/' + stage + '.xml')}")
        cases = list(ET.parse(output / (stage + ".xml")).getroot().iter("testcase"))
        counts = {"tests": len(cases), "skipped": sum(c.find("skipped") is not None for c in cases),
                  "failures": sum(c.find("failure") is not None or c.find("error") is not None for c in cases)}
        record["tests"][package] = counts
        save()
        if not cases or counts["skipped"] or counts["failures"]:
            raise RuntimeError(f"{package}: missing, skipped, or failed tests: {counts}")

    try:
        save()
        run("image", ["docker", "image", "inspect", args.image])
        run("gpu", ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv"])
        run("container", ["docker", "run", "-d", "--rm", "--gpus", "all", "--name", name,
                          "--network", "none", "-e", "PYTHONDONTWRITEBYTECODE=1",
                          "-v", f"{ROOT}:/workspace:rw", "-w", "/workspace",
                          "--entrypoint", "sleep", args.image, "infinity"])
        inside("build", f"BUILD_JOBS={args.jobs} BUILD_WORKERS=1 CUDA_ARCHITECTURES={shlex.quote(args.architecture)} "
               "DUOJIN_CLEAN_BUILD=ON DUOJIN_ENABLE_CUDA=ON DUOJIN_CUDA_REQUIRED=ON ./scripts/build_cuda_local.sh")
        # Freeze source and installed bytes immediately after the build, before executing fixtures.
        record["source_built"] = manifest()
        if record["source_built"] != record["source_before"]:
            raise RuntimeError("Sources changed during build; rerun in a new evidence directory")
        record["artifacts"] = artifact_manifest("cuda")
        record["compiler_flags"] = {
            str(p.relative_to(ROOT)): p.read_text()
            for p in sorted((ROOT / "build/cuda-local-v3").rglob("flags.make"))
        }
        save()
        for package in PACKAGES:
            ctest(package)
        inside("python", setup + "python3 -m pytest -q tests --ignore=tests/integration --ignore=tests/performance")
        for launch in ("localization", "navigation", "nav2", "navigation_stack", "online_navigation"):
            inside("launch-" + launch, setup + f"ros2 launch duojin02_bringup {launch}.launch.py --show-args")
        inside("localization-launch", setup + "python3 tests/integration/cuda_localization_launch_probe.py "
               f"--output {shlex.quote(inside_output + '/localization-launch')}")
        inside("mppi-probe-build", setup + "cmake -S tests/integration/cuda_core_probe "
               "-B build/cuda-core-probe -DCMAKE_BUILD_TYPE=Release && cmake --build build/cuda-core-probe --clean-first -j2")
        inside("mppi-probe", setup + "python3 tests/integration/ros_isolation.py "
               "build/cuda-core-probe/cuda_mppi_core_plugin_probe "
               "/workspace/src/duojin02_bringup/config/nav2.yaml /workspace/install/cuda-local-v3 "
               + shlex.quote(inside_output + "/mppi-probe.json"))
        inside("amcl-replay", setup + "python3 tests/integration/cuda_amcl_replay_probe.py "
               "--executable /workspace/install/cuda-local-v3/lib/duojin02_amcl_cuda/amcl "
               "--expected-prefix /workspace/install/cuda-local-v3 "
               "--parameters /workspace/src/duojin02_bringup/config/nav2.yaml "
               f"--frames {args.frames} --output {shlex.quote(inside_output + '/amcl-replay')}")
        if args.cpu_image:
            run("cpu-image", ["docker", "image", "inspect", args.cpu_image])
            run("cpu-container", ["docker", "run", "-d", "--rm", "--name", name + "-cpu",
                                  "--network", "none", "-e", "PYTHONDONTWRITEBYTECODE=1",
                                  "-v", f"{ROOT}:/workspace:rw", "-w", "/workspace",
                                  "--entrypoint", "sleep", args.cpu_image, "infinity"])

            def cpu(stage, command):
                run(stage, ["docker", "exec", name + "-cpu", "bash", "-c", command])

            cpu("cpu-no-toolkit", "! command -v nvcc && test ! -e /dev/nvidiactl")
            cpu("cpu-build", f"BUILD_JOBS={args.jobs} BUILD_WORKERS=1 DUOJIN_CLEAN_BUILD=ON DUOJIN_ENABLE_CUDA=OFF "
                "DUOJIN_CUDA_REQUIRED=OFF ./scripts/build_cuda_local.sh")
            record["cpu_artifacts"] = artifact_manifest("cpu")
            cpu_setup = "export DUOJIN_ENABLE_CUDA=OFF; source /workspace/scripts/source_cuda_local.sh && "
            for package in PACKAGES:
                stage = package + "-cpu-tests"
                cpu(stage, cpu_setup + f"ctest --test-dir build/cpu-local-v3/{package} "
                    "-E '^precise_mppi_gpu_limit_checks$' --no-tests=error --output-on-failure "
                    f"--output-junit {shlex.quote(inside_output + '/' + stage + '.xml')}")
                cases = list(ET.parse(output / (stage + ".xml")).getroot().iter("testcase"))
                if not cases or any(c.find("skipped") is not None or c.find("failure") is not None or
                                    c.find("error") is not None for c in cases):
                    raise RuntimeError(f"{stage}: missing/skipped/failed CPU tests")
                record["tests"][package + "-cpu"] = {"tests": len(cases), "skipped": 0, "failures": 0}
            cpu("cpu-localization-launch", cpu_setup + "python3 tests/integration/cuda_localization_launch_probe.py "
                f"--output {shlex.quote(inside_output + '/cpu-localization-launch')}")
            record["cpu_artifacts_after"] = artifact_manifest("cpu")
            if record["cpu_artifacts_after"] != record["cpu_artifacts"]:
                raise RuntimeError("CPU install changed during tests")
        record["artifacts_after"] = artifact_manifest("cuda")
        if record["artifacts_after"] != record["artifacts"]:
            raise RuntimeError("CUDA install changed during tests")
        record["source_after"] = manifest()
        if record["source_after"] != record["source_built"]:
            raise RuntimeError("Sources changed during tests; rerun in a new evidence directory")
        record["status"] = "passed"
    except Exception as error:
        record["status"], record["error"] = "failed", str(error)
        print(str(error), file=sys.stderr)
    finally:
        record["finished_utc"] = datetime.now(timezone.utc).isoformat()
        save()
        subprocess.run(["docker", "rm", "-f", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if args.cpu_image:
            subprocess.run(["docker", "rm", "-f", name + "-cpu"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print(f"{record['status']}: {output / 'validation.json'}")
    return 0 if record["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
