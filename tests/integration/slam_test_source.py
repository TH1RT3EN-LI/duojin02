"""Resolve and verify already-prepared SLAM test source without network or writes."""

import argparse
import importlib.util
from pathlib import Path
import sys


PACKAGED_SOURCE = Path("/opt/duojin02/test_sources/slam_toolbox")


def verify(root, source):
    # Share production provenance checks, including a complete source tree digest for
    # stacked patches. This remains read-only and never fetches a source archive.
    spec = importlib.util.spec_from_file_location(
        "prepare_slam_backend", Path(__file__).resolve().parents[2] / "scripts/prepare_slam_backend.py"
    )
    backend = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(backend)
    backend.verify_prepared_source(root, source)
    if not (source / "solvers/ceres_solver.cpp").is_file():
        raise ValueError("prepared source is missing solvers/ceres_solver.cpp")

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
        "Use the local Docker environment, or prepare it with "
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
