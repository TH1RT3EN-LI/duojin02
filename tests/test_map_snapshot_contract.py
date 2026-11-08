import json
from pathlib import Path
import sys
import struct
import zlib

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/duojin02_slam_tools"))
from duojin02_slam_tools.snapshots import (
    artifact_path,
    commit_snapshot,
    reserve_snapshot,
    validate_map,
)


def png_pixel():
    def chunk(kind, payload):
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", zlib.crc32(kind + payload))
        )

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", 1, 1, 8, 0, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(b"\x00\xfe"))
        + chunk(b"IEND", b"")
    )


def snapshot(directory, name):
    with reserve_snapshot(directory, name) as (pending, final):
        artifact_path(pending, ".yaml").write_text(
            f"image: {name}.pgm\nresolution: 0.02\norigin: [0, 0, 0]\n"
        )
        artifact_path(pending, ".pgm").write_bytes(b"P5\n1 1\n255\n\xfe")
        artifact_path(pending, ".posegraph").write_bytes(b"graph")
        artifact_path(pending, ".data").write_bytes(b"scans")
        return commit_snapshot(pending, final, {"fixture": True})


def test_foreign_manifest_cannot_verify_another_selected_map(tmp_path):
    first, second = snapshot(tmp_path, "first"), snapshot(tmp_path, "second")
    second.with_suffix(".snapshot.json").write_bytes(
        first.with_suffix(".snapshot.json").read_bytes()
    )
    second.with_suffix(".pgm").write_bytes(b"changed unverified raster")
    with pytest.raises(ValueError, match="does not match the selected"):
        validate_map(second)


def test_image_role_must_bind_to_the_image_referenced_by_the_yaml(tmp_path):
    selected = snapshot(tmp_path, "selected")
    other = snapshot(tmp_path, "other")
    manifest_file = selected.with_suffix(".snapshot.json")
    manifest = json.loads(manifest_file.read_text())
    manifest["files"]["image"] = json.loads(other.with_suffix(".snapshot.json").read_text())[
        "files"
    ]["image"]
    manifest_file.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match="role image does not match"):
        validate_map(selected)


@pytest.mark.parametrize("files", [{}, [], None, {"yaml": {}}, {"yaml": None}])
def test_declared_snapshot_cannot_omit_required_file_roles(tmp_path, files):
    selected = snapshot(tmp_path, "selected")
    selected.with_suffix(".snapshot.json").write_text(json.dumps({"schema": 1, "files": files}))
    with pytest.raises(ValueError, match="snapshot"):
        validate_map(selected)


@pytest.mark.parametrize("artifact", [".posegraph", ".data"])
def test_raster_consumer_ignores_unused_graph_damage_but_continuation_rejects_it(
    tmp_path, artifact
):
    selected = snapshot(tmp_path, "selected")
    selected.with_suffix(artifact).write_bytes(b"damaged graph")
    validate_map(selected)
    with pytest.raises(ValueError, match="modified"):
        validate_map(selected, require_graph=True)


@pytest.mark.parametrize("role", ["posegraph", "data"])
def test_declared_continuation_snapshot_requires_both_graph_roles(tmp_path, role):
    selected = snapshot(tmp_path, "selected")
    manifest_file = selected.with_suffix(".snapshot.json")
    manifest = json.loads(manifest_file.read_text())
    del manifest["files"][role]
    manifest_file.write_text(json.dumps(manifest))
    validate_map(selected)
    with pytest.raises(ValueError, match="Missing or invalid snapshot role"):
        validate_map(selected, require_graph=True)


@pytest.mark.parametrize(
    "extension,payload",
    [
        ("pgm", b"P5\n1 1\n255\n\xfe"),
        ("pnm", b"P2\n1 1\n255\n254\n"),
        ("png", png_pixel()),
    ],
)
def test_legacy_maps_without_manifest_keep_supported_image_formats(tmp_path, extension, payload):
    image = tmp_path / ("legacy." + extension)
    image.write_bytes(payload)
    selected = tmp_path / "legacy.yaml"
    selected.write_text(f"image: {image.name}\nresolution: 0.02\norigin: [0, 0, 0]\n")
    assert validate_map(selected)[1] == image


@pytest.mark.parametrize(
    "geometry", ["resolution: true\norigin: [0, 0, 0]", "resolution: 0.02\norigin: [false, 0, 0]"]
)
def test_boolean_values_are_not_real_map_geometry(tmp_path, geometry):
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\n1 1\n255\n\xfe")
    selected = tmp_path / "map.yaml"
    selected.write_text(f"image: map.pgm\n{geometry}\n")
    with pytest.raises(ValueError, match="Invalid map"):
        validate_map(selected)


def test_scan_geometry_sidecar_is_published_hashed_and_checked(tmp_path):
    with reserve_snapshot(tmp_path, "deskew") as (pending, final):
        artifact_path(pending, ".yaml").write_text("image: deskew.pgm\nresolution: 0.01\norigin: [-26.125937054897271, -12.140692030483075, 0]\n")
        artifact_path(pending, ".pgm").write_bytes(b"P5\n1 1\n255\n\xfe")
        artifact_path(pending, ".posegraph").write_bytes(b"graph")
        artifact_path(pending, ".data").write_bytes(b"scans")
        contract = {"schema": "duojin02.scan_geometry.v1", "scan_motion_compensation": True}
        artifact_path(pending, ".scan_geometry.json").write_text(json.dumps(contract))
        selected = commit_snapshot(pending, final, {"slam_parameters": {"scan_motion_compensation": True}})
    manifest = json.loads(selected.with_suffix(".snapshot.json").read_text())
    assert manifest["files"]["scan_geometry"]["name"] == "deskew.scan_geometry.json"
    validate_map(selected, require_graph=True)
    selected.with_suffix(".scan_geometry.json").write_text("{}")
    with pytest.raises(ValueError, match="modified snapshot"):
        validate_map(selected)


@pytest.mark.parametrize("parameter,value", [("scan_motion_compensation", True), ("capture_time_offset_sec", .01), ("scan_reference_fraction", .5), ("capture_time_mapping_id", "custom-v1"), ("capture_time_mapping_source", "device-log")])
def test_new_motion_or_time_mapping_cannot_silently_drop_sidecar(tmp_path, parameter, value):
    with reserve_snapshot(tmp_path, "missing") as (pending, final):
        artifact_path(pending, ".yaml").write_text("image: missing.pgm\nresolution: 0.01\norigin: [0, 0, 0]\n")
        artifact_path(pending, ".pgm").write_bytes(b"P5\n1 1\n255\n\xfe")
        with pytest.raises(ValueError, match="requires a scan_geometry sidecar"):
            commit_snapshot(pending, final, {"slam_parameters": {parameter: value}}, include_graph=False)
    assert not list(tmp_path.glob("missing.*"))


def test_raster_only_export_preserves_complete_read_only_geometry_contract(tmp_path):
    parameters = {
        "scan_motion_compensation": True, "capture_time_offset_sec": .015,
        "capture_time_mapping_id": "independent-calibration-v1",
        "capture_time_mapping_source": "device-log",
        "scan_reference_fraction": .5, "scan_max_tf_gap_sec": .1,
    }
    with reserve_snapshot(tmp_path, "raster") as (pending, final):
        artifact_path(pending, ".yaml").write_text("image: raster.pgm\nresolution: 0.01\norigin: [0, 0, 0]\n")
        artifact_path(pending, ".pgm").write_bytes(b"P5\n1 1\n255\n\xfe")
        selected = commit_snapshot(pending, final, {"slam_parameters": parameters}, include_graph=False)
    from duojin02_bringup.scan_geometry import contract
    assert json.loads(selected.with_suffix(".scan_geometry.json").read_text()) == contract(parameters)
    validate_map(selected)
