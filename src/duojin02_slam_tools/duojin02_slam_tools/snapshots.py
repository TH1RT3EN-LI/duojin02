import ctypes
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
from .scan_contract import contract as scan_contract, SCHEMA as SCAN_GEOMETRY_SCHEMA


def digest(path):
    checksum = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def artifact_path(prefix, extension):
    return Path(str(prefix) + extension)


def validate_map(path, verify_snapshot=True, require_graph=False):
    path = Path(path)
    metadata = yaml.safe_load(path.read_text())
    if not isinstance(metadata, dict) or not isinstance(metadata.get("image"), str):
        raise ValueError(f"Invalid map YAML: {path}")
    resolution = metadata.get("resolution")
    origin = metadata.get("origin")
    if type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError(f"Invalid map resolution: {path}")
    if (
        not isinstance(origin, list)
        or len(origin) != 3
        or not all(type(value) in (int, float) and math.isfinite(value) for value in origin)
    ):
        raise ValueError(f"Invalid map origin: {path}")
    image = Path(metadata["image"])
    image = image if image.is_absolute() else path.parent / image
    if not image.is_file() or image.stat().st_size == 0:
        raise ValueError(f"Map image is missing or empty: {image}")
    manifest_path = path.with_suffix(".snapshot.json")
    bundle_manifest = path.parent / "snapshot.json"
    if verify_snapshot and bundle_manifest.exists():
        manifest, roles = validate_bundle(path.parent, require_graph=require_graph)
        if roles["yaml"].resolve() != path.resolve() or roles["image"].resolve() != image.resolve():
            raise ValueError("Selected raster does not match the snapshot bundle")
        return metadata, image.resolve()
    if verify_snapshot and manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if (
            not isinstance(manifest, dict)
            or type(manifest.get("schema")) is not int
            or manifest["schema"] != 1
        ):
            raise ValueError(f"Unsupported map snapshot schema: {manifest_path}")
        files = manifest.get("files")
        if not isinstance(files, dict):
            raise ValueError(f"Invalid map snapshot file roles: {manifest_path}")
        required = {"yaml": path, "image": image}
        geometry_path = path.with_suffix(".scan_geometry.json")
        if geometry_path.exists() or "scan_geometry" in files:
            required["scan_geometry"] = geometry_path
        if require_graph:
            required.update(
                posegraph=path.with_suffix(".posegraph"), data=path.with_suffix(".data")
            )
        for role, expected in required.items():
            entry = files.get(role)
            if not isinstance(entry, dict) or not isinstance(entry.get("name"), str):
                raise ValueError(f"Missing or invalid snapshot role {role}: {manifest_path}")
            artifact = path.parent / entry["name"]
            if artifact.parent.resolve() != path.parent.resolve():
                raise ValueError("Snapshot artifacts must be in the map directory")
            if artifact.resolve() != expected.resolve():
                raise ValueError(
                    f"Snapshot role {role} does not match the selected map artifact: {expected}"
                )
            if not artifact.is_file() or digest(artifact) != entry.get("sha256"):
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
            identifiers = [
                int(path.name.split(".", 1)[0])
                for path in directory.iterdir()
                if path.name.split(".", 1)[0].isdigit()
            ]
            name = str(max(identifiers, default=-1) + 1)
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", name):
            raise ValueError("map_name must be a filename without directories")
        if (directory / name).exists() or any(directory.glob(f"{name}.*")):
            raise FileExistsError(f"Map {name} already exists; choose a new map_name")
        with tempfile.TemporaryDirectory(prefix=f".{name}.pending-", dir=directory) as temporary:
            yield Path(temporary) / name, directory / name


def commit_snapshot(
    staged_prefix, final_prefix, provenance, include_graph=True, validate_raster=False
):
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
        files.update(
            posegraph=artifact_path(staged_prefix, ".posegraph"),
            data=artifact_path(staged_prefix, ".data"),
        )
    geometry_path = artifact_path(staged_prefix, ".scan_geometry.json")
    parameters = provenance.get("slam_parameters", {})
    geometry_required = parameters.get("scan_motion_compensation", False) or bool(
        parameters.get("capture_time_offset_sec", 0.0)
    ) or bool(parameters.get("scan_reference_fraction", 0.0)) or (
        parameters.get("capture_time_mapping_id", "header-first-ray-assumed-v1") !=
        "header-first-ray-assumed-v1"
    ) or bool(parameters.get("capture_time_mapping_source", ""))
    # Raster-only saves do not call SerializePoseGraph. Read-only runtime geometry
    # parameters still define the same contract, so retain it without a graph.
    contract_keys = (
        "scan_motion_compensation", "capture_time_offset_sec", "capture_time_mapping_id",
        "capture_time_mapping_source", "scan_reference_fraction", "scan_max_tf_gap_sec",
    )
    if not include_graph and not geometry_path.exists() and all(key in parameters for key in contract_keys):
        contract = scan_contract(parameters)
        geometry_path.write_text(json.dumps(contract, indent=2, allow_nan=False) + "\n")
    if geometry_path.exists() or geometry_required:
        if not geometry_path.is_file():
            raise ValueError("Scan motion/time mapping requires a scan_geometry sidecar")
        contract = json.loads(geometry_path.read_text())
        if not isinstance(contract, dict) or contract.get("schema") not in ("duojin02.scan_geometry.v1", SCAN_GEOMETRY_SCHEMA):
            raise ValueError("Unsupported scan_geometry sidecar schema")
        files["scan_geometry"] = geometry_path
    for artifact in files.values():
        if not artifact.is_file() or artifact.stat().st_size == 0:
            raise ValueError(f"Map saver did not produce a complete snapshot: {artifact}")
    manifest = {
        "schema": 1,
        "map_id": final_prefix.name,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "provenance": provenance,
        "files": {
            key: {
                "name": value.name,
                "bytes": value.stat().st_size,
                "sha256": digest(value),
            }
            for key, value in files.items()
        },
    }
    if raster_validation is not None:
        manifest["raster_validation"] = raster_validation
    manifest_path = artifact_path(staged_prefix, ".snapshot.json")
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

    order = [value for key, value in files.items() if key != "yaml"] + [
        manifest_path,
        staged_yaml,
    ]
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


def validate_bundle(directory, *, require_graph=True):
    """Verify one native immutable graph/raster bundle, including its identity."""
    directory = Path(directory)
    manifest = json.loads((directory / "snapshot.json").read_text())
    if (not isinstance(manifest, dict) or manifest.get("schema_version") != 2
            or manifest.get("consistency") != "single_graph_owner_boundary"
            or manifest.get("producer") not in ("slam_toolbox.SaveSnapshot.v1", "slam_toolbox.SaveSnapshot.v2")):
        raise ValueError("Unsupported snapshot bundle")
    operation = manifest.get("operation")
    request_id = manifest.get("request_id")
    if (operation not in ("snapshot", "finalize") or not isinstance(request_id, str)
            or type(manifest.get("finalized")) is not bool
            or manifest["finalized"] != (operation == "finalize")
            or (operation == "finalize") != bool(request_id)):
        raise ValueError("Invalid snapshot operation/request identity")
    entries = manifest.get("files")
    if not isinstance(entries, list):
        raise ValueError("Invalid bundle file entries")
    roles, names = {}, set()
    records = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("Invalid bundle file record")
        role, name = entry.get("role"), entry.get("path")
        if (not isinstance(role, str) or not role or role in roles
                or not isinstance(name, str) or not name or name in names
                or Path(name).name != name or name in (".", "..", "snapshot.json")):
            raise ValueError("Invalid or duplicate bundle role/path")
        artifact = directory / name
        size = entry.get("size_bytes")
        checksum = entry.get("sha256")
        if (artifact.is_symlink() or not artifact.is_file() or type(size) is not int
                or size <= 0 or artifact.stat().st_size != size
                or not isinstance(checksum, str) or not re.fullmatch(r"[a-f0-9]{64}", checksum)
                or digest(artifact) != checksum):
            raise ValueError(f"Incomplete or modified bundle artifact: {artifact}")
        roles[role] = artifact
        names.add(name)
        records.append((role, f"{role}\t{name}\t{checksum}\t{size}\n"))
    required = {"yaml", "image", "graph_update", "scan_geometry", "input_boundary", "runtime"}
    if manifest["producer"] == "slam_toolbox.SaveSnapshot.v2":
        required.add("laser_bindings")
    if require_graph or operation == "finalize":
        required |= {"posegraph", "data"}
    if not required <= roles.keys():
        raise ValueError(f"Missing bundle roles: {sorted(required - roles.keys())}")
    if "laser_bindings" in roles and roles["laser_bindings"].name != "laser_bindings.json":
        raise ValueError("Unsupported laser bindings bundle path")
    if "laser_bindings" not in roles and (directory / "laser_bindings.json").exists():
        raise ValueError("Unmanifested laser bindings in snapshot bundle")
    if ("posegraph" in roles) != ("data" in roles):
        raise ValueError("Partial graph archive in snapshot")
    identity = "".join(record for _, record in sorted(records)) + f"{operation}\n{request_id}\n"
    if hashlib.sha256(identity.encode()).hexdigest() != manifest.get("snapshot_id"):
        raise ValueError("Snapshot identity does not match its members")
    boundary = json.loads(roles["input_boundary"].read_text())
    if operation == "finalize" and (boundary.get("admissions_paused") is not True
                                    or boundary.get("queues_drained") is not True):
        raise ValueError("Finalize lacks a paused/drained input boundary")
    return manifest, roles


def graph_input(path):
    """Return the native Load argument for either a bundle or legacy prefix."""
    path = Path(path)
    directory = path if path.is_dir() else path.parent
    if (directory / "snapshot.json").is_file():
        validate_bundle(directory, require_graph=True)
        return directory
    return path.with_suffix("") if path.suffix == ".posegraph" else path


def commit_bundle_snapshot(staged, final, provenance=None, *, require_graph=True):
    staged, final = Path(staged), Path(final)
    manifest, roles = validate_bundle(staged, require_graph=require_graph)
    metadata, image = validate_map(roles["yaml"], require_graph=require_graph)
    validate_trinary_raster(metadata, image)
    # Native bundles are immutable. Client observations are a separate receipt,
    # bound to the native manifest, and never rewrite the captured state.
    receipt = {
        "schema": "duojin02.snapshot_receipt.v1",
        "snapshot_id": manifest["snapshot_id"],
        "native_manifest_sha256": digest(staged / "snapshot.json"),
        "client_observed_provenance": provenance or {},
    }
    native = ctypes.CDLL(None, use_errno=True)
    rename = native.renameat2
    rename.argtypes = (ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint)
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(staged), -100, os.fsencode(final), 1):
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error), str(final))
    descriptor = os.open(final.parent, os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    receipt_path = final.with_name(final.name + ".receipt.json")
    with receipt_path.open("x") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    descriptor = os.open(final.parent, os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return final / roles["yaml"].name
