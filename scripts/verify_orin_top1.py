#!/usr/bin/env python3
"""Verify the pinned Orin Top1 profile and overlay without loading ROS or CUDA."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import platform
import re
import sys

PROFILE_PATH = "src/duojin01_bringup/config/chassis.yaml"
INSTALL_SUBDIR = "install/orin_top1"
MANIFEST_PATH = "docs/evidence/orin-top1/release.json"
REQUIRED_LIBRARIES = frozenset({
    "duojin01_navigation/lib/libprecise_mppi.so",
    "duojin01_navigation/lib/libduojin_gpu_backend.so",
    "duojin01_navigation/lib/libprofile_transport.so",
    "duojin01_navigation/lib/libstudy_inflation_layer.so",
})
REQUIRED_METADATA = frozenset({
    "local_setup.bash",
    "_local_setup_util_sh.py",
    "duojin01_navigation/share/duojin01_navigation/controller_plugins.xml",
    "duojin01_navigation/share/duojin01_navigation/study_costmap_plugins.xml",
    "duojin01_navigation/share/duojin01_navigation/goal_checker_plugins.xml",
    "duojin01_navigation/share/duojin01_navigation/planner_plugins.xml",
    "duojin01_navigation/share/duojin01_navigation/package.xml",
    "duojin01_navigation/share/ament_index/resource_index/packages/duojin01_navigation",
    "duojin01_navigation/share/ament_index/resource_index/nav2_core__pluginlib__plugin/duojin01_navigation",
    "duojin01_navigation/share/ament_index/resource_index/nav2_costmap_2d__pluginlib__plugin/duojin01_navigation",
})
RUNTIME_ENVIRONMENT_KEYS = frozenset({
    "OMP_NUM_THREADS", "DUOJIN_GPU_COST_KERNEL",
    "DUOJIN_GPU_ROLLOUT_THREADS", "DUOJIN_GPU_COST_THREADS",
})


class ReleaseError(ValueError):
    """The selected release cannot be admitted."""


def contained_path(base: Path, relative: str) -> Path:
    """Refuse lexical traversal and symlinks that escape the selected root."""
    if not isinstance(relative, str) or not relative or "\\" in relative:
        raise ReleaseError("invalid relative file path")
    parts = relative.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise ReleaseError(f"unsafe relative file path: {relative}")
    if PurePosixPath(relative).is_absolute():
        raise ReleaseError(f"absolute file path refused: {relative}")
    candidate = (base / relative).resolve(strict=True)
    if not candidate.is_relative_to(base):
        raise ReleaseError(f"file escapes its selected root: {relative}")
    return candidate


def sha256(path: Path) -> str:
    if not path.is_file():
        raise ReleaseError(f"required file is not a regular file: {path.name}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_digest(path: Path, expected: object, label: str) -> str:
    if not isinstance(expected, str) or re.fullmatch(r"[0-9a-f]{64}", expected) is None:
        raise ReleaseError(f"invalid SHA-256 for {label}")
    actual = sha256(path)
    if actual != expected:
        raise ReleaseError(f"SHA-256 mismatch: {label}")
    return actual


def verify_release(root: Path, *, files_only: bool = False) -> dict:
    root = root.resolve(strict=True)
    manifest_file = contained_path(root, MANIFEST_PATH)
    manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or type(manifest.get("schema_version")) is not int or manifest["schema_version"] != 1:
        raise ReleaseError("unsupported release manifest schema")
    if manifest.get("architecture") != "aarch64" or manifest.get("cuda_architecture") != "sm87":
        raise ReleaseError("release must target aarch64 / sm87")
    machine = platform.machine().lower()
    if not files_only and machine != "aarch64":
        raise ReleaseError(f"Top1 activation requires aarch64, found {machine}")
    if manifest.get("install_subdir") != INSTALL_SUBDIR:
        raise ReleaseError("unexpected overlay install directory")
    if not isinstance(manifest.get('runtime_environment'), dict) or set(manifest['runtime_environment']) != RUNTIME_ENVIRONMENT_KEYS:
        raise ReleaseError("runtime environment keys differ from the admitted contract")
    profile = manifest.get("profile")
    if not isinstance(profile, dict) or profile.get("path") != PROFILE_PATH:
        raise ReleaseError("unexpected Top1 profile path")
    profile_hash = verify_digest(contained_path(root, PROFILE_PATH), profile.get("sha256"), PROFILE_PATH)
    if manifest.get('configuration_schema') != 1:
        raise ReleaseError('single chassis configuration schema is required')
    # Load this workspace's renderer rather than an inherited installed module.
    import importlib.util
    renderer_path = contained_path(root, 'src/duojin01_bringup/duojin01_bringup/chassis_profile.py')
    spec = importlib.util.spec_from_file_location('admitted_chassis_profile', renderer_path)
    renderer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)
    selected = renderer.load_profile(root / PROFILE_PATH)
    if selected['runtime_environment'] != manifest['runtime_environment']:
        raise ReleaseError('runtime environment differs from the chassis authority')
    hashes = {name: __import__('hashlib').sha256(renderer.payload(selected, name)).hexdigest()
              for name in sorted(renderer.COMPONENTS | {'lidar', 'slam_offline'})}
    if profile.get('derived_component_sha256') != hashes:
        raise ReleaseError('derived chassis component hash mismatch')
    install_root = contained_path(root, INSTALL_SUBDIR)
    if not install_root.is_dir():
        raise ReleaseError("overlay install root is not a directory")
    required_files = manifest.get("required_install_files")
    if not isinstance(required_files, dict):
        raise ReleaseError("required_install_files must be a path-to-SHA-256 object")
    required = REQUIRED_LIBRARIES | REQUIRED_METADATA
    missing = required - required_files.keys()
    if missing:
        raise ReleaseError("manifest omits required files: " + ", ".join(sorted(missing)))
    for relative, expected in sorted(required_files.items()):
        verify_digest(contained_path(install_root, relative), expected, relative)
    return {
        "ok": True,
        "architecture": machine,
        "architecture_checked": not files_only,
        "verified_install_files": len(required_files),
        "profile_sha256": profile_hash,
        "configuration_schema": 1,
        "verified_components": len(hashes),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--files-only", action="store_true", help="skip host architecture check for an offline file audit")
    args = parser.parse_args(argv)
    try:
        result = verify_release(args.root, files_only=args.files_only)
    except (OSError, ValueError, TypeError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
