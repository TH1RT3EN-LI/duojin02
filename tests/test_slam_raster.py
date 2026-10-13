"""Unknown occupancy survives new saves; failed snapshots stay unpublished."""

import json
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/duojin01_slam_tools"))
from duojin01_slam_tools.raster import validate_trinary_raster
from duojin01_slam_tools.snapshots import artifact_path, commit_snapshot, reserve_snapshot


def staged(prefix, free=0.196, pixels=b"\x00\xcd\xfe", graph=True):
    metadata = dict(image=prefix.name + ".pgm", mode="trinary", resolution=0.02,
                    origin=[-1.234567, -2.345678, 0], negate=0,
                    occupied_thresh=0.65, free_thresh=free)
    artifact_path(prefix, ".yaml").write_text(yaml.safe_dump(metadata))
    artifact_path(prefix, ".pgm").write_bytes(b"P5\n# saved map\n3 1\n255\n" + pixels)
    if graph:
        artifact_path(prefix, ".posegraph").write_bytes(b"graph")
        artifact_path(prefix, ".data").write_bytes(b"scans")
    return metadata


def test_three_classes_reload_and_origin_remains_unchanged(tmp_path):
    with reserve_snapshot(tmp_path, "new") as (pending, final):
        metadata = staged(pending)
        path = commit_snapshot(pending, final, {}, validate_raster=True)
    saved = yaml.safe_load(path.read_text())
    assert saved["origin"] == metadata["origin"]
    manifest = json.loads(path.with_suffix(".snapshot.json").read_text())
    assert manifest["raster_validation"]["loaded_class_counts"] == {
        "occupied": 1, "unknown": 1, "free": 1}
    assert manifest["raster_validation"]["origin_modified"] is False


def test_unsafe_new_save_is_rejected_and_a_later_good_save_can_succeed(tmp_path):
    old_yaml = tmp_path / "old.yaml"
    old_yaml.write_text("existing user map\n")
    old_bytes = old_yaml.read_bytes()
    with reserve_snapshot(tmp_path, "new") as (pending, final):
        staged(pending, free=0.25)
        with pytest.raises(ValueError, match="preserve unknown"):
            commit_snapshot(pending, final, {}, validate_raster=True)
    assert not list(tmp_path.glob("new.*"))
    assert old_yaml.read_bytes() == old_bytes
    with reserve_snapshot(tmp_path, "new") as (pending, final):
        staged(pending)
        assert commit_snapshot(pending, final, {}, validate_raster=True).exists()
    assert old_yaml.read_bytes() == old_bytes


@pytest.mark.parametrize("pixels, expected", [(b"\x00\xcd", "Truncated"),
                                               (b"\x00\xcc\xfe", "unexpected grayscale")])
def test_bad_raster_is_not_published(tmp_path, pixels, expected):
    with reserve_snapshot(tmp_path, "new") as (pending, final):
        staged(pending, pixels=pixels)
        with pytest.raises(ValueError, match=expected):
            commit_snapshot(pending, final, {}, validate_raster=True)
    assert not list(tmp_path.glob("new.*"))


def test_successful_raster_does_not_publish_if_graph_failed(tmp_path):
    with reserve_snapshot(tmp_path, "new") as (pending, final):
        staged(pending, graph=False)
        with pytest.raises(ValueError, match="complete"):
            commit_snapshot(pending, final, {}, validate_raster=True)
    assert not list(tmp_path.glob("new.*"))


def test_crlf_header_and_whitespace_first_raster_byte_are_not_trimmed(tmp_path):
    # A forbidden gray value must be rejected rather than silently trimmed.
    image = tmp_path / "map.pgm"
    image.write_bytes(b"P5\r\n3 1\r\n255\r\n\x20\xcd\xfe")
    with pytest.raises(ValueError, match="unexpected grayscale"):
        validate_trinary_raster(dict(mode="trinary", negate=0,
                                    free_thresh=0.196, occupied_thresh=0.65), image)
