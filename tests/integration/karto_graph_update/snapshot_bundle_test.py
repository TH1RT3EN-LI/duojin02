import hashlib
import json
from pathlib import Path

import pytest

from duojin02_slam_tools.snapshots import (
    commit_bundle_snapshot, digest, graph_input, reserve_snapshot,
    validate_bundle, validate_map,
)


def fixture(directory, *, finalized=False, producer="slam_toolbox.SaveSnapshot.v1", laser_bindings=False):
    directory.mkdir()
    contents = {
        "yaml": ("map.yaml", b"image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"),
        "image": ("map.pgm", b"P5\n1 1\n255\n\xfe"),
        "posegraph": ("graph.posegraph", b"archived graph"),
        "data": ("graph.data", b"archived data"),
        "graph_update": ("graph_update.json", b'{"graph_revision":1}'),
        "scan_geometry": ("scan_geometry.json", b'{"schema":"test"}'),
        "runtime": ("runtime.json", b'{"source_marker_binding":"not_verified_by_snapshot"}'),
        "input_boundary": ("input_boundary.json", b'{"admissions_paused":true,"queues_drained":true}'),
    }
    if laser_bindings:
        contents["laser_bindings"] = ("laser_bindings.json", b'{"schema":"slam-laser-bindings.v1","history_policy":"synthetic-fixture","frames":[]}')
    entries = []
    for role, (name, content) in sorted(contents.items()):
        member = directory / name
        member.write_bytes(content)
        entries.append(dict(role=role, path=name, sha256=digest(member), size_bytes=len(content)))
    operation, request_id = ("finalize", "declared-request-1") if finalized else ("snapshot", "")
    identity = "".join(f'{e["role"]}\t{e["path"]}\t{e["sha256"]}\t{e["size_bytes"]}\n' for e in entries) + f"{operation}\n{request_id}\n"
    manifest = dict(schema_version=2, producer=producer, snapshot_id=hashlib.sha256(identity.encode()).hexdigest(), operation=operation, request_id=request_id, finalized=finalized, consistency="single_graph_owner_boundary", files=entries)
    (directory / "snapshot.json").write_text(json.dumps(manifest))
    return manifest


def test_one_bundle_commits_without_rewriting_native_members(tmp_path):
    with reserve_snapshot(tmp_path, "new-map") as (pending, final):
        original = fixture(pending)
        hashes = {item.name: digest(item) for item in pending.iterdir()}
        path = commit_bundle_snapshot(pending, final, {"component": "client-observed"})
    assert path == final / "map.yaml"
    assert {item.name: digest(item) for item in final.iterdir()} == hashes
    assert validate_bundle(final)[0] == original
    assert validate_map(path, require_graph=True)[1] == (final / "map.pgm").resolve()
    assert graph_input(path) == final
    assert json.loads(final.with_name(final.name + ".receipt.json").read_text())["snapshot_id"] == original["snapshot_id"]
    with pytest.raises(FileExistsError):
        with reserve_snapshot(tmp_path, "new-map"):
            pass


def test_bundle_rejects_modified_graph_before_publication(tmp_path):
    staged = tmp_path / "staged"
    fixture(staged)
    (staged / "graph.data").write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="modified"):
        commit_bundle_snapshot(staged, tmp_path / "final")
    assert not (tmp_path / "final").exists()


def test_bundle_refuses_overwrite_even_empty_directory(tmp_path):
    staged, final = tmp_path / "staged", tmp_path / "final"
    fixture(staged)
    final.mkdir()
    with pytest.raises(FileExistsError):
        commit_bundle_snapshot(staged, final)
    assert staged.is_dir() and not list(final.iterdir())


def test_v2_binds_laser_registry_sidecar_to_immutable_bundle(tmp_path):
    directory = tmp_path / "bundle"
    fixture(directory, producer="slam_toolbox.SaveSnapshot.v2", laser_bindings=True)
    manifest, roles = validate_bundle(directory)
    assert manifest["producer"] == "slam_toolbox.SaveSnapshot.v2"
    assert roles["laser_bindings"] == directory / "laser_bindings.json"
    roles["laser_bindings"].write_text('{"modified":true}')
    with pytest.raises(ValueError, match="modified"):
        validate_bundle(directory)


def test_v2_cannot_claim_complete_device_identity_without_binding_member(tmp_path):
    directory = tmp_path / "bundle"
    fixture(directory, producer="slam_toolbox.SaveSnapshot.v2")
    with pytest.raises(ValueError, match="Missing bundle roles.*laser_bindings"):
        validate_bundle(directory)


@pytest.mark.parametrize("producer", ["slam_toolbox.SaveSnapshot.v1", "slam_toolbox.SaveSnapshot.v2"])
def test_binding_role_layout_matches_native_reader_even_after_identity_reseal(tmp_path, producer):
    directory = tmp_path / "bundle"
    manifest = fixture(directory, producer=producer, laser_bindings=True)
    (directory / "laser_bindings.json").rename(directory / "renamed.json")
    for entry in manifest["files"]:
        if entry["role"] == "laser_bindings":
            entry["path"] = "renamed.json"
    identity = "".join(f'{e["role"]}\t{e["path"]}\t{e["sha256"]}\t{e["size_bytes"]}\n' for e in manifest["files"]) + "snapshot\n\n"
    manifest["snapshot_id"] = hashlib.sha256(identity.encode()).hexdigest()
    (directory / "snapshot.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="laser bindings bundle path"):
        validate_bundle(directory)


def test_legacy_bundle_cannot_supply_device_identity_outside_its_manifest(tmp_path):
    directory = tmp_path / "bundle"
    fixture(directory)
    validate_bundle(directory)  # Ordinary v1 migration remains supported.
    (directory / "laser_bindings.json").write_text('{"unbound":true}')
    with pytest.raises(ValueError, match="Unmanifested laser bindings"):
        validate_bundle(directory)


@pytest.mark.parametrize("mutation", ["duplicate", "escape", "symlink", "identity", "finalize-boundary"])
def test_bundle_verifies_members_identity_and_finalize_boundary(tmp_path, mutation):
    directory = tmp_path / "bundle"
    manifest = fixture(directory, finalized=mutation == "finalize-boundary")
    if mutation == "duplicate":
        manifest["files"].append(manifest["files"][0])
    elif mutation == "escape":
        manifest["files"][0]["path"] = "../outside"
    elif mutation == "symlink":
        target = directory / "graph.data"
        outside = tmp_path / "outside"
        target.replace(outside)
        target.symlink_to(outside)
    elif mutation == "identity":
        manifest["snapshot_id"] = "0" * 64
    else:
        boundary = directory / "input_boundary.json"
        boundary.write_text('{"admissions_paused":true,"queues_drained":false}')
        # Recompute member hash and identity: logical boundary still must fail.
        for entry in manifest["files"]:
            if entry["role"] == "input_boundary":
                entry.update(sha256=digest(boundary), size_bytes=boundary.stat().st_size)
        identity = "".join(f'{e["role"]}\t{e["path"]}\t{e["sha256"]}\t{e["size_bytes"]}\n' for e in manifest["files"]) + f'finalize\n{manifest["request_id"]}\n'
        manifest["snapshot_id"] = hashlib.sha256(identity.encode()).hexdigest()
    (directory / "snapshot.json").write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        validate_bundle(directory)
