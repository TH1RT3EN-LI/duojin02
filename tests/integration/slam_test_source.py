"""Resolve and verify already-prepared SLAM test source without network or writes."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import os
import sys


PACKAGED_SOURCE = Path("/opt/duojin02/test_sources/slam_toolbox")


def verify(root, source):
    patch_directory = root / "patches/slam_toolbox"
    provenance = json.loads((patch_directory / "source.json").read_text())
    patch = patch_directory / provenance["patch"]
    expected = {**provenance, "patch_sha256": hashlib.sha256(patch.read_bytes()).hexdigest()}
    if json.loads((source / "hardware_build.json").read_text()) != expected:
        raise ValueError("source revision or patch checksum differs from the current checkout")
    if not (source / "solvers/ceres_solver.cpp").is_file():
        raise ValueError("prepared source is missing solvers/ceres_solver.cpp")
    environment = {**os.environ, "GIT_CEILING_DIRECTORIES": str(source.parent)}
    for key in ("GIT_DIR", "GIT_WORK_TREE", "GIT_COMMON_DIR", "GIT_INDEX_FILE"):
        environment.pop(key, None)
    result = subprocess.run(
        ["git", "apply", "--no-index", "--reverse", "--check", str(patch)],
        cwd=source,
        env=environment,
        capture_output=True,
        text=True,
    )
    if result.returncode:
        raise ValueError(
            f"prepared source does not contain the current patch: {result.stderr.strip()}"
        )
    return source


def resolve_source(root, source=None):
    candidates = (
        [source] if source else [root / ".cache/slam_backend/src/slam_toolbox", PACKAGED_SOURCE]
    )
    failures = []
    for candidate in candidates:
        candidate = candidate.resolve()
        try:
            return verify(root, candidate)
        except (OSError, ValueError, KeyError) as error:
            failures.append(f"{candidate}: {error}")
    raise RuntimeError(
        "Offline SLAM tests require verified, patched source already present. "
        "Use the Docker testing target, or prepare it with "
        "scripts/prepare_slam_backend.py before entering the offline test environment. "
        "PREPARED_SLAM_SOURCE may select an existing source directory.\n" + "\n".join(failures)
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--source", type=Path)
    args = parser.parse_args()
    try:
        print(resolve_source(args.root.resolve(), args.source))
    except RuntimeError as error:
        print(error, file=sys.stderr)
        raise SystemExit(1)
