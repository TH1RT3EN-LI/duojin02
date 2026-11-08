"""Reject stale or incomplete installation provenance without requiring ROS."""
import json
import os
from pathlib import Path
import platform
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/cuda_install_provenance.py"
PACKAGES = ("duojin02_scan_geometry", "duojin02_mppi_controller", "duojin02_amcl_cuda",
            "duojin02_navigation", "duojin02_bringup", "duojin02_slam_tools")


def run(*args):
    return subprocess.run([sys.executable, "-B", str(SCRIPT), *map(str, args)],
                          capture_output=True, text=True, env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})


def fixture(tmp_path):
    for package in PACKAGES:
        directory = tmp_path / "src" / package
        directory.mkdir(parents=True)
        (directory / "package.xml").write_text(package)
    prefix = tmp_path / "install"
    prefix.mkdir()
    before = tmp_path / "before.json"
    result = run("snapshot", "--root", tmp_path)
    assert result.returncode == 0, result.stderr
    before.write_text(result.stdout)
    return prefix, before


def test_source_changes_additions_and_missing_manifest_are_rejected(tmp_path):
    prefix, before = fixture(tmp_path)
    assert run("verify", "--root", tmp_path, "--prefix", prefix).returncode != 0
    assert run("seal", "--root", tmp_path, "--before", before, "--prefix", prefix).returncode == 0
    assert run("verify", "--root", tmp_path, "--prefix", prefix).returncode == 0
    changed = tmp_path / "src" / PACKAGES[0] / "new.cpp"
    changed.write_text("changed")
    assert "new.cpp" in run("verify", "--root", tmp_path, "--prefix", prefix).stderr
    assert run("seal", "--root", tmp_path, "--before", before, "--prefix", prefix).returncode != 0
    changed.unlink()
    (tmp_path / "src" / PACKAGES[0] / "package.xml").unlink()
    assert run("verify", "--root", tmp_path, "--prefix", prefix).returncode != 0


def test_python_caches_do_not_invalidate_build(tmp_path):
    prefix, before = fixture(tmp_path)
    cache = tmp_path / "src" / PACKAGES[0] / "__pycache__"
    cache.mkdir()
    (cache / "module.pyc").write_bytes(b"generated")
    assert run("seal", "--root", tmp_path, "--before", before, "--prefix", prefix).returncode == 0
    assert json.loads((prefix / ".duojin02-source-manifest.json").read_text())["schema"] == 1
    assert run("verify", "--root", tmp_path, "--prefix", prefix).returncode == 0


def test_backend_rejects_missing_or_mismatched_patches_and_geometry(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "scripts"))
    from prepare_slam_backend import source_provenance

    prefix = tmp_path / "backend"
    prefix.mkdir()
    assert run("verify-backend", "--prefix", prefix).returncode != 0
    (prefix / ".target_architecture").write_text(platform.machine() + "\n")
    scan = run("snapshot", "--scope", "scan")
    assert scan.returncode == 0
    (prefix / ".duojin02-scan-source.json").write_text(scan.stdout)
    for name in ("slam_toolbox", "nav2_map_server"):
        _, _, provenance = source_provenance(ROOT, name)
        directory = prefix / "share" / name
        directory.mkdir(parents=True)
        (directory / "hardware_build.json").write_text(json.dumps(provenance))
    assert run("verify-backend", "--prefix", prefix).returncode == 0
    marker = prefix / "share/slam_toolbox/hardware_build.json"
    bad = json.loads(marker.read_text())
    bad["commit"] = "stale"
    marker.write_text(json.dumps(bad))
    assert "slam_toolbox" in run("verify-backend", "--prefix", prefix).stderr
