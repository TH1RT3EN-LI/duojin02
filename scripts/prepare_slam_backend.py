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


def prepare(root: Path, destination: Path) -> Path:
    patch_dir = root / "patches/slam_toolbox"
    source = json.loads((patch_dir / "source.json").read_text())
    patch = patch_dir / source["patch"]
    provenance = {**source, "patch_sha256": hashlib.sha256(patch.read_bytes()).hexdigest()}
    marker = destination / "hardware_build.json"
    if (
        marker.is_file()
        and json.loads(marker.read_text()) == provenance
        and apply_patch(destination, patch, "--reverse", "--check", check=False).returncode == 0
    ):
        return destination

    destination.parent.mkdir(parents=True, exist_ok=True)
    archive = destination.parent / f"slam_toolbox-{source['commit']}.tar.gz"
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
        extracted = next(staging.iterdir())
        apply_patch(extracted, patch, "--check")
        apply_patch(extracted, patch)
        apply_patch(extracted, patch, "--reverse", "--check")
        (extracted / "hardware_build.json").write_text(json.dumps(provenance, indent=2) + "\n")
        if destination.exists():
            shutil.rmtree(destination)
        extracted.replace(destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--destination", type=Path)
    args = parser.parse_args()
    destination = args.destination or args.root / ".cache/slam_backend/src/slam_toolbox"
    print(prepare(args.root.resolve(), destination.resolve()))


if __name__ == "__main__":
    main()
