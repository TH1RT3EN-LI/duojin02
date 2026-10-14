import hashlib
import importlib.util
from pathlib import Path

import pytest

from test_slam_backend_preparation import backend, make_cached_backend


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "slam_test_source", ROOT / "tests/integration/slam_test_source.py"
)
preflight = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preflight)


def prepared_source(tmp_path):
    destination, patch = make_cached_backend(tmp_path)
    backend.prepare(tmp_path, destination)
    (destination / "solvers").mkdir()
    (destination / "solvers/ceres_solver.cpp").write_text("// fixture\n")
    return destination, patch


def test_existing_source_is_verified_without_mutation(tmp_path):
    source, _ = prepared_source(tmp_path)
    before = {
        path.relative_to(source): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source.rglob("*")
        if path.is_file()
    }
    assert preflight.resolve_source(tmp_path, source) == source
    after = {
        path.relative_to(source): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source.rglob("*")
        if path.is_file()
    }
    assert after == before


def test_false_provenance_cannot_hide_an_unapplied_patch(tmp_path):
    source, _ = prepared_source(tmp_path)
    (source / "main.cpp").write_text("old\n")
    with pytest.raises(RuntimeError, match="does not contain the current patch"):
        preflight.resolve_source(tmp_path, source)


def test_current_patch_checksum_must_match(tmp_path):
    source, patch = prepared_source(tmp_path)
    patch.write_text(patch.read_text().replace("patched", "updated"))
    with pytest.raises(RuntimeError, match="patch checksum differs"):
        preflight.resolve_source(tmp_path, source)


def test_offline_missing_source_reports_the_preparation_boundary(tmp_path, monkeypatch):
    source, _ = make_cached_backend(tmp_path)
    monkeypatch.setattr(preflight, "PACKAGED_SOURCE", tmp_path / "missing-image-source")
    with pytest.raises(RuntimeError, match="before entering the offline test environment"):
        preflight.resolve_source(tmp_path)
    assert not source.exists()


def test_packaged_source_is_a_verified_fallback(tmp_path, monkeypatch):
    source, _ = prepared_source(tmp_path)
    monkeypatch.setattr(preflight, "PACKAGED_SOURCE", source)
    assert preflight.resolve_source(tmp_path) == source
