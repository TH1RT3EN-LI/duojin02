#!/usr/bin/env python3
"""Record/check project package inputs and the selected patched SLAM manifests."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import tempfile

from prepare_slam_backend import source_provenance

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = ("duojin02_scan_geometry", "duojin02_mppi_controller", "duojin02_amcl_cuda",
            "duojin02_navigation", "duojin02_bringup", "duojin02_slam_tools")
MARKER = ".duojin02-source-manifest.json"


def snapshot(root, packages=PACKAGES):
    files = {}
    for package in packages:
        directory = root / "src" / package
        if not directory.is_dir():
            raise ValueError(f"missing source package: {directory}")
        for path in sorted(directory.rglob("*")):
            if "__pycache__" in path.parts or path.suffix in (".pyc", ".pyo"):
                continue
            if path.is_file():
                files[path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {"schema": 1, "files": files}


def verify_backend(root, prefix):
    if (prefix / ".target_architecture").read_text().strip() != platform.machine():
        raise ValueError("SLAM backend was built for a different architecture")
    scan_manifest = json.loads((prefix / ".duojin02-scan-source.json").read_text())
    if scan_manifest != snapshot(root, ("duojin02_scan_geometry",)):
        raise ValueError("SLAM backend scan geometry sources differ from the build")
    for dependency in ("slam_toolbox", "nav2_map_server"):
        _, _, expected = source_provenance(root, dependency)
        marker_path = prefix / "share" / dependency / "hardware_build.json"
        installed = json.loads(marker_path.read_text())
        installed.pop("source_tree_sha256", None)
        if installed != expected:
            raise ValueError(f"{dependency} installation does not match current revision/patches: {marker_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("snapshot", "seal", "verify", "verify-backend"))
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--prefix", type=Path)
    parser.add_argument("--before", type=Path)
    parser.add_argument("--scope", choices=("all", "scan"), default="all")
    args = parser.parse_args()
    try:
        if args.action == "snapshot":
            packages = PACKAGES if args.scope == "all" else ("duojin02_scan_geometry",)
            print(json.dumps(snapshot(args.root, packages), sort_keys=True))
        elif args.action == "verify-backend":
            verify_backend(args.root, args.prefix)
        elif args.action == "seal":
            before = json.loads(args.before.read_text())
            if before != snapshot(args.root):
                raise ValueError("source changed during build; rebuild before using this install")
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=args.prefix,
                                             prefix=".manifest-", delete=False) as tmp:
                try:
                    json.dump(before, tmp, sort_keys=True)
                    tmp.write("\n")
                    tmp.flush()
                    os.fchmod(tmp.fileno(), 0o644)
                    os.replace(tmp.name, args.prefix / MARKER)
                except BaseException:
                    os.unlink(tmp.name)
                    raise
        else:
            marker = json.loads((args.prefix / MARKER).read_text())
            current = snapshot(args.root)
            if marker != current:
                before, after = marker.get("files", {}), current["files"]
                changed = sorted(k for k in before.keys() | after.keys() if before.get(k) != after.get(k))
                raise ValueError("package sources differ from build: " + ", ".join(changed[:5]))
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        print(f"Installation provenance check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
