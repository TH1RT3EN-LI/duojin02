#!/usr/bin/env python3
"""Fetch a verified Humble source revision and apply the reviewed project patch."""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request
from pathlib import Path


def apply_patch(directory, patch, *arguments, check=True):

    environment = dict(os.environ, GIT_CEILING_DIRECTORIES=str(directory.parent))
    for name in ("GIT_DIR", "GIT_WORK_TREE", "GIT_COMMON_DIR", "GIT_INDEX_FILE"):
        environment.pop(name, None)
    result = subprocess.run(
        ["git", "apply", "--no-index", *arguments, str(patch)],
        cwd=directory,
        env=environment,
        capture_output=True,
        text=True,
    )
    if check and result.returncode:
        raise RuntimeError(f"SLAM patch verification failed: {result.stderr.strip()}")
    return result


def source_tree_sha256(directory: Path) -> str:
    """Hash the complete prepared source, including paths, excluding our marker."""
    digest = hashlib.sha256()
    for path in sorted(directory.rglob("*")):
        if path.is_file() and path.name != "hardware_build.json":
            digest.update(path.relative_to(directory).as_posix().encode() + b"\0")
            digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def source_provenance(root: Path, dependency="slam_toolbox"):
    patch_dir = root / "patches" / dependency
    source = json.loads((patch_dir / "source.json").read_text())
    # Apply only manifest-ordered, unique filenames confined to the patch directory.
    names = source["patches"] if "patches" in source else [source["patch"]]
    if not isinstance(names, list) or not names:
        raise ValueError("SLAM patch list must be nonempty, ordered, and unique")
    if any(not isinstance(name, str) or Path(name).name != name for name in names):
        raise ValueError("SLAM patch entries must be filenames in the dependency patch directory")
    if len(set(names)) != len(names):
        raise ValueError("SLAM patch list must be nonempty, ordered, and unique")
    patches = [patch_dir / name for name in names]
    provenance = {**source, "patch_sha256": hashlib.sha256(patches[0].read_bytes()).hexdigest()}
    if "patches" in source:
        provenance["patches_sha256"] = {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in patches
        }
    return source, patches, provenance


def verify_prepared_source(root: Path, destination: Path, dependency="slam_toolbox"):
    _, patches, provenance = source_provenance(root, dependency)
    marker = json.loads((destination / "hardware_build.json").read_text())
    tree_digest = marker.pop("source_tree_sha256", None)
    if marker != provenance:
        raise ValueError("source revision or patch checksum differs from the current checkout")
    if tree_digest:
        if source_tree_sha256(destination) != tree_digest:
            raise ValueError("prepared source differs from its verified source tree")
    elif len(patches) == 1:
        if apply_patch(destination, patches[0], "--reverse", "--check", check=False).returncode:
            raise ValueError("prepared source does not contain the current patch")
    else:
        raise ValueError("multi-patch source requires a verified source tree digest")
    return destination


def prepare(root: Path, destination: Path, dependency="slam_toolbox") -> Path:
    source, patches, provenance = source_provenance(root, dependency)
    try:
        return verify_prepared_source(root, destination, dependency)
    except (OSError, ValueError, KeyError):
        pass

    destination.parent.mkdir(parents=True, exist_ok=True)
    archive = destination.parent / f"{dependency}-{source['commit']}.tar.gz"
    if (
        not archive.is_file()
        or hashlib.sha256(archive.read_bytes()).hexdigest() != source["archive_sha256"]
    ):
        partial = archive.with_suffix(".download")
        with (
            urllib.request.urlopen(source["archive_url"], timeout=120) as response,
            partial.open("wb") as output,
        ):
            shutil.copyfileobj(response, output)
        if hashlib.sha256(partial.read_bytes()).hexdigest() != source["archive_sha256"]:
            partial.unlink()
            raise RuntimeError("SLAM Toolbox source checksum does not match the pinned revision")
        partial.replace(archive)

    with tempfile.TemporaryDirectory(dir=destination.parent) as directory:
        staging = Path(directory)
        with tarfile.open(archive) as contents:
            for member in contents.getmembers():
                resolved = (staging / member.name).resolve()
                if staging.resolve() not in resolved.parents:
                    raise RuntimeError("Invalid source archive member")
            contents.extractall(staging)
        extracted = next(staging.iterdir()) / source.get("archive_subdirectory", "")
        for patch in patches:
            apply_patch(extracted, patch, "--check")
            apply_patch(extracted, patch)
        if "patches" in source:
            provenance["source_tree_sha256"] = source_tree_sha256(extracted)
        else:
            apply_patch(extracted, patches[0], "--reverse", "--check")
        (extracted / "hardware_build.json").write_text(json.dumps(provenance, indent=2) + "\n")
        if destination.exists():
            shutil.rmtree(destination)
        extracted.replace(destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--destination", type=Path)
    parser.add_argument(
        "--dependency", choices=("slam_toolbox", "nav2_map_server"), default="slam_toolbox"
    )
    args = parser.parse_args()
    destination = args.destination or args.root / ".cache/slam_backend/src" / args.dependency
    print(prepare(args.root.resolve(), destination.resolve(), args.dependency))


if __name__ == "__main__":
    main()
