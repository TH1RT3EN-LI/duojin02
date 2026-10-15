#!/usr/bin/env python3
"""Reproduce local RTX CUDA checks and paired API timing in an isolated ROS container."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
import re
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_manifest() -> dict:
    files = sorted(set(
        list((ROOT / "src/duojin02_navigation").rglob("*"))
        + list((ROOT / "src/duojin02_bringup/config").rglob("*"))
        + list((ROOT / "tests/performance").rglob("*"))
        + list((ROOT / "scripts").glob("*.sh")) + list((ROOT / "scripts").glob("*.py"))
        + [ROOT / "tests/integration/cuda_local_plugin_probe.cpp", ROOT / "Dockerfile",
           ROOT / "tests/run_suites.py", ROOT / "tests/test_test_entrypoint.py"]
    ))
    hashes = {str(path.relative_to(ROOT)): sha256(path) for path in files if path.is_file()}
    digest = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    return {"combined_sha256": digest, "files": hashes}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default="duojin02:hardware-humble")
    parser.add_argument("--samples", type=int, default=40)
    parser.add_argument("--jobs", type=int, default=6)
    parser.add_argument("--architecture", default="native")
    parser.add_argument("--output", type=Path, help="New evidence directory inside this project; defaults to a unique UTC run directory")
    parser.add_argument("--baseline", type=Path, default=ROOT.parent / "duojin02")
    parser.add_argument("--skip-navigation", action="store_true", help="Geometry-only validation; records omitted navigation coverage")
    args = parser.parse_args()
    if args.samples < 2 or args.jobs < 1:
        parser.error("samples must be >= 2 and jobs >= 1")
    if args.output is None:
        epoch = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:6]
        args.output = ROOT / "output/cuda-local-v2" / ("validation-" + epoch)
    args.output = args.output.resolve()
    if (args.output / "validation.json").exists():
        parser.error("--output already contains validation.json; use a new directory to preserve evidence")
    if not args.baseline.is_dir():
        parser.error("--baseline must be an existing directory; it is mounted read-only")
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        relative_output = args.output.relative_to(ROOT)
    except ValueError:
        parser.error("--output must be inside this project")
    container_output = "/workspace/" + str(relative_output)
    quoted_output = shlex.quote(container_output)
    name = "duojin02-cuda-local-" + uuid.uuid4().hex[:10]
    record = {
        "schema_version": 1, "status": "running", "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_before": source_manifest(),
        "baseline_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "image": args.image, "architecture": args.architecture, "commands": [],
        "scope": "local desktop CUDA; geometry API and installed controller plugin; no robot/closed-loop claims",
        "navigation_omitted": args.skip_navigation,
        "clock_note": "Monotonic wall durations; shared GPU desktop workload is not isolated",
    }
    summary = args.output / "validation.json"
    resource_file = (args.output / "gpu-resources.csv").open("w")
    resource_monitor = None

    def save():
        summary.write_text(json.dumps(record, indent=2) + "\n")

    def run(stage: str, command: list[str], *, expect: int = 0):
        print(f"[{stage}] {shlex.join(command)}", flush=True)
        begin = time.monotonic()
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (args.output / (stage + ".log")).write_text(result.stdout)
        print(result.stdout, end="", flush=True)
        record["commands"].append({"stage": stage, "argv": command, "returncode": result.returncode,
                                   "expected_returncode": expect, "wall_seconds": time.monotonic() - begin})
        save()
        if result.returncode != expect:
            raise RuntimeError(f"{stage}: exit {result.returncode}, expected {expect}")
        return result.stdout

    def inside(stage: str, command: str, *, expect: int = 0):
        return run(stage, ["docker", "exec", name, "bash", "-c", command], expect=expect)

    def ctest(stage: str, build: str, pattern: str = ""):
        junit = f"{container_output}/{stage}.xml"
        command = f"ctest --test-dir {build} --output-on-failure --no-tests=error --output-junit {shlex.quote(junit)}"
        if pattern:
            command += " -R " + shlex.quote(pattern)
        if "navigation" in build:
            command = "source /opt/ros/humble/setup.bash && source /opt/duojin02/source.sh && " + f"export LD_LIBRARY_PATH=/workspace/{build}/gpu:${{LD_LIBRARY_PATH}} && " + command
        inside(stage, command)
        xml = ET.parse(args.output / (stage + ".xml")).getroot()
        cases = list(xml.iter("testcase"))
        skipped = sum(case.find("skipped") is not None for case in cases)
        failures = sum(case.find("failure") is not None or case.find("error") is not None for case in cases)
        record.setdefault("tests", {})[stage] = {"tests": len(cases), "skipped": skipped, "failures": failures}
        save()
        if not cases or skipped or failures:
            raise RuntimeError(f"{stage}: zero tests, skips, or failures are not accepted")

    try:
        save()
        record["image_inspect"] = json.loads(run("image-inspect", ["docker", "image", "inspect", args.image]))[0]
        run("host-gpu", ["nvidia-smi", "--query-gpu=name,driver_version,memory.total,memory.used,pstate,clocks.sm,clocks.mem,power.draw", "--format=csv"])
        run("container-create", ["docker", "run", "-d", "--rm", "--gpus", "all", "--name", name,
             "-v", f"{ROOT}:/workspace:rw", "-v", f"{args.baseline.resolve()}:/baseline:ro",
             "-w", "/workspace", "--entrypoint", "sleep", args.image, "infinity"])
        inside("toolchain", "nvcc --version && cmake --version && c++ --version && nvidia-smi")
        common = f"-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_CUDA_ARCHITECTURES={shlex.quote(args.architecture)}"
        inside("strict-cuda-configure", f"cmake -S src/duojin02_navigation/gpu -B build/local-cuda {common} -DDUOJIN_CUDA_REQUIRED=ON -DDUOJIN_ENABLE_CUDA=ON")
        inside("strict-cuda-build", f"cmake --build build/local-cuda -j {args.jobs} --target all gpu_geometry_benchmark")
        ctest("strict-cuda-tests", "build/local-cuda")
        # A required-CUDA executable returning 77 must fail under CTest, never appear as a pass.
        inside("required-device-negative", "CUDA_VISIBLE_DEVICES= build/local-cuda/gpu_geometry_checks --require-cuda", expect=77)
        inside("required-device-ctest-negative", f"CUDA_VISIBLE_DEVICES= ctest --test-dir build/local-cuda -R '^gpu_geometry_cuda_checks$' --output-on-failure --output-junit {quoted_output}/required-device-negative.xml", expect=8)
        negative_cases = list(ET.parse(args.output / "required-device-negative.xml").getroot().iter("testcase"))
        if len(negative_cases) != 1 or negative_cases[0].find("failure") is None or negative_cases[0].find("skipped") is not None:
            raise RuntimeError("Strict CTest missing-device case did not fail explicitly")
        inside("cpu-configure", "cmake -S src/duojin02_navigation/gpu -B build/local-cpu -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DDUOJIN_ENABLE_CUDA=OFF -DDUOJIN_CUDA_REQUIRED=OFF")
        inside("cpu-build", f"cmake --build build/local-cpu -j {args.jobs}")
        ctest("cpu-tests", "build/local-cpu")
        # Configuration must reject silent CPU fallback when a compiler is missing.
        inside("required-compiler-negative", "cmake -S src/duojin02_navigation/gpu -B build/local-no-compiler -DDUOJIN_ENABLE_CUDA=ON -DDUOJIN_CUDA_REQUIRED=ON -DCMAKE_CUDA_COMPILER=NOTFOUND", expect=1)
        resource_monitor = subprocess.Popen(["nvidia-smi", "--query-gpu=timestamp,name,memory.used,pstate,clocks.sm,clocks.mem,power.draw,utilization.gpu", "--format=csv", "--loop-ms=100"], stdout=resource_file, stderr=subprocess.DEVNULL)
        record["gpu_resource_sampling"] = {
            "file": "gpu-resources.csv", "period_ms": 100,
            "timestamp_timezone": str(datetime.now().astimezone().tzinfo),
            "utc_offset_seconds": datetime.now().astimezone().utcoffset().total_seconds(),
            "scope": "whole GPU including desktop; sampled peak is not exact process VRAM peak",
            "window": "benchmark through navigation build/tests/probe and binary hashing; excludes earlier standalone checks",
        }
        inside("paired-benchmark", f"build/local-cuda/gpu_geometry_benchmark --require-cuda --samples {args.samples} --json-out {quoted_output}/geometry-benchmark.json")
        if not args.skip_navigation:
            ros = "source /opt/ros/humble/setup.bash && source /opt/duojin02/source.sh && "
            navigation_build = "build/local-navigation"
            install = "/workspace/install/cuda-local"
            inside("navigation-configure", ros + f"cmake -S src/duojin02_navigation -B {navigation_build} {common} -DDUOJIN_CUDA_REQUIRED=ON -DDUOJIN_ENABLE_CUDA=ON -DCMAKE_INSTALL_PREFIX={install} -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3")
            inside("navigation-build", ros + f"cmake --build {navigation_build} -j {args.jobs}")
            inside("navigation-install", f"cmake --install {navigation_build}")
            ctest("navigation-checks", navigation_build, "gpu_geometry|precise_mppi|geometry_checks|tracking_checks|velocity_limit|scan_validity|guard_input_validity")
            inside("navigation-cpu-configure", ros + "cmake -S src/duojin02_navigation -B build/local-navigation-cpu -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DDUOJIN_ENABLE_CUDA=OFF -DDUOJIN_CUDA_REQUIRED=OFF -DPython3_EXECUTABLE=/usr/bin/python3")
            inside("navigation-cpu-build", ros + f"cmake --build build/local-navigation-cpu -j {args.jobs} --target precise_mppi_limit_checks")
            ctest("navigation-cpu-checks", "build/local-navigation-cpu", "^precise_mppi_(limit|no_device)_checks$")
            # Source the new install last; the probe verifies actual mapped libraries.
            plugin_log = inside("installed-plugin-probe", ros + f"source {install}/share/duojin02_navigation/local_setup.bash && export LD_LIBRARY_PATH={install}/lib:${{LD_LIBRARY_PATH}} && {navigation_build}/cuda_local_plugin_probe /workspace/src/duojin02_bringup/config/nav2.yaml {install} {quoted_output}/plugin-probe.json")
            summaries = re.findall(r"CUDA_LOCAL_GEOMETRY_BRANCHES ([^\n]+)", plugin_log)
            branch_counts = [{key: int(value) for key, value in re.findall(r"(\w+)=(\d+)", line)} for line in summaries]
            record["plugin_branch_summaries"] = branch_counts
            for branch in ("direct_cuda_batches", "centering_cuda_batches", "braking_cuda_batches"):
                if not branch_counts or max(line.get(branch, 0) for line in branch_counts) <= 0:
                    raise RuntimeError(f"Installed plugin did not execute required CUDA branch: {branch}")
        hashes = inside("binary-hashes", "find build/local-cuda build/local-cpu build/local-navigation build/local-navigation-cpu install/cuda-local -type f \\( -name '*.so' -o -name 'gpu_geometry_checks' -o -name 'gpu_geometry_benchmark' -o -name 'precise_mppi_limit_checks' -o -name 'cuda_local_plugin_probe' -o -name 'CMakeCache.txt' \\) -exec sha256sum {} + 2>/dev/null" + (" || test $? = 1" if args.skip_navigation else ""))
        record["build_files"] = {line.split(None, 1)[1]: line.split(None, 1)[0] for line in hashes.splitlines() if re.match(r"^[a-f0-9]{64}  ", line)}
        if not record["build_files"]:
            raise RuntimeError("No binary hashes were captured")
        run("host-gpu-after", ["nvidia-smi", "--query-gpu=name,driver_version,memory.total,memory.used,pstate,clocks.sm,clocks.mem,power.draw", "--format=csv"])
        record["source_after"] = source_manifest()
        record["source_unchanged_during_validation"] = record["source_before"] == record["source_after"]
        if not record["source_unchanged_during_validation"]:
            raise RuntimeError("Source files changed during validation; rerun to freeze valid evidence")
        record["compiler_flags"] = {
            str(path.relative_to(ROOT)): path.read_text()
            for path in (ROOT / "build").glob("local-*/**/flags.make")
            if path.parent.name in ("duojin02_gpu_geometry.dir", "precise_mppi.dir")
        }
        record["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        record["status"] = "passed"
        save()
        print(f"Validation passed: {summary}", flush=True)
        return 0
    except Exception as error:
        record["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        record["status"] = "failed"
        record["error"] = str(error)
        save()
        print(str(error), file=sys.stderr)
        return 1
    finally:
        if resource_monitor is not None:
            resource_monitor.terminate()
            resource_monitor.wait()
        resource_file.close()
        subprocess.run(["docker", "exec", name, "chown", "-R", f"{os.getuid()}:{os.getgid()}", "/workspace/build/local-cuda", "/workspace/build/local-cpu", "/workspace/build/local-navigation", "/workspace/build/local-navigation-cpu", "/workspace/install/cuda-local", container_output], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["docker", "rm", "-f", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    sys.exit(main())
