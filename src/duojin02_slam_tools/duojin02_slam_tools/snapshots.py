"""Publish a complete map snapshot with the YAML file as the commit marker."""

import fcntl
import hashlib
import json
import math
import os
import re
import tempfile
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

import yaml

from .raster import validate_trinary_raster


def digest(path):
    checksum = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def artifact_path(prefix, extension):
    return Path(str(prefix) + extension)


def validate_map(path, verify_snapshot=True):
    path = Path(path)
    metadata = yaml.safe_load(path.read_text())
    if not isinstance(metadata, dict) or not isinstance(metadata.get("image"), str):
        raise ValueError(f"Invalid map YAML: {path}")
    resolution = metadata.get("resolution")
    origin = metadata.get("origin")
    if not isinstance(resolution, (int, float)) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError(f"Invalid map resolution: {path}")
    if not isinstance(origin, list) or len(origin) != 3 or not all(
        isinstance(value, (int, float)) and math.isfinite(value) for value in origin
    ):
        raise ValueError(f"Invalid map origin: {path}")
    image = Path(metadata["image"])
    image = image if image.is_absolute() else path.parent / image
    if not image.is_file() or image.stat().st_size == 0:
        raise ValueError(f"Map image is missing or empty: {image}")
    manifest_path = path.with_suffix(".snapshot.json")
    if verify_snapshot and manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("schema") != 1:
            raise ValueError(f"Unsupported map snapshot schema: {manifest_path}")
        for entry in manifest["files"].values():
            artifact = path.parent / entry["name"]
            if artifact.parent.resolve() != path.parent.resolve():
                raise ValueError("Snapshot artifacts must be in the map directory")
            if not artifact.is_file() or digest(artifact) != entry["sha256"]:
                raise ValueError(f"Incomplete or modified snapshot artifact: {artifact}")
    return metadata, image.resolve()


@contextmanager
def reserve_snapshot(directory, name, *, deadline=None):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / ".save-map.lock").open("a") as lock:
        while True:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if deadline is None or time.monotonic() >= deadline:
                    raise TimeoutError("Another map save is still in progress")
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        if name in ("", "auto"):
            identifiers = [int(path.name.split(".", 1)[0]) for path in directory.iterdir()
                           if path.name.split(".", 1)[0].isdigit()]
            name = str(max(identifiers, default=-1) + 1)
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", name):
            raise ValueError("map_name must be a filename without directories")
        if any(directory.glob(f"{name}.*")):
            raise FileExistsError(f"Map {name} already exists; choose a new map_name")
        with tempfile.TemporaryDirectory(prefix=f".{name}.pending-", dir=directory) as temporary:
            yield Path(temporary) / name, directory / name


def commit_snapshot(staged_prefix, final_prefix, provenance, include_graph=True, validate_raster=False):
    staged_prefix, final_prefix = Path(staged_prefix), Path(final_prefix)
    staged_yaml = artifact_path(staged_prefix, ".yaml")
    metadata, image = validate_map(staged_yaml, verify_snapshot=False)
    if image.parent != staged_prefix.parent.resolve():
        raise ValueError("Map saver wrote the image outside the pending snapshot")
    raster_validation = validate_trinary_raster(metadata, image) if validate_raster else None
    metadata["image"] = image.name
    staged_yaml.write_text(yaml.safe_dump(metadata, sort_keys=False))
    files = {"image": image, "yaml": staged_yaml}
    if include_graph:
        files.update(posegraph=artifact_path(staged_prefix, ".posegraph"),
                     data=artifact_path(staged_prefix, ".data"))
    for artifact in files.values():
        if not artifact.is_file() or artifact.stat().st_size == 0:
            raise ValueError(f"Map saver did not produce a complete snapshot: {artifact}")
    manifest = {
        "schema": 1, "map_id": final_prefix.name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "provenance": provenance,
        "files": {key: {"name": value.name, "bytes": value.stat().st_size,
                        "sha256": digest(value)} for key, value in files.items()},
    }
    if raster_validation is not None:
        manifest["raster_validation"] = raster_validation
    manifest_path = artifact_path(staged_prefix, ".snapshot.json")
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    # Consumers discover *.yaml. Publish it last, after every referenced file is durable.
    order = [value for key, value in files.items() if key != "yaml"] + [manifest_path, staged_yaml]
    published = []
    try:
        for artifact in order:
            if artifact == staged_yaml:
                descriptor = os.open(final_prefix.parent, os.O_DIRECTORY)
                try:
                    os.fsync(descriptor)
                finally:
                    os.close(descriptor)
            with artifact.open("rb") as stream:
                os.fsync(stream.fileno())
            target = final_prefix.parent / artifact.name
            if target.exists():
                raise FileExistsError(f"Refusing to overwrite {target}")
            artifact.replace(target)
            published.append(target)
        descriptor = os.open(final_prefix.parent, os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    except Exception:
        for target in reversed(published):
            target.unlink(missing_ok=True)
        raise
    return artifact_path(final_prefix, ".yaml")
