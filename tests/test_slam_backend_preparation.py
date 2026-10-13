"""Ensure nested Git caches contain the patch their provenance claims."""

import hashlib
import importlib.util
import io
import json
import subprocess
import tarfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('prepare_slam_backend', ROOT / 'scripts/prepare_slam_backend.py')
backend = importlib.util.module_from_spec(spec)
spec.loader.exec_module(backend)


def make_cached_backend(tmp_path):
    subprocess.run(['git', 'init', '-q', '--initial-branch=fixture'], cwd=tmp_path, check=True)
    patch_dir = tmp_path / 'patches/slam_toolbox'
    patch_dir.mkdir(parents=True)
    patch = patch_dir / 'hardware.patch'
    patch.write_text('--- a/main.cpp\n+++ b/main.cpp\n@@ -1 +1 @@\n-old\n+patched\n')
    destination = tmp_path / '.cache/backend/slam_toolbox'
    destination.parent.mkdir(parents=True)
    archive = destination.parent / 'slam_toolbox-fixture.tar.gz'
    with tarfile.open(archive, 'w:gz') as contents:
        member = tarfile.TarInfo('slam_toolbox-fixture/main.cpp')
        member.size = 4
        contents.addfile(member, io.BytesIO(b'old\n'))
    source = {'version': 'fixture', 'commit': 'fixture', 'archive_url': archive.as_uri(),
              'archive_sha256': hashlib.sha256(archive.read_bytes()).hexdigest(), 'patch': patch.name}
    (patch_dir / 'source.json').write_text(json.dumps(source))
    return destination, patch


def test_nested_git_cache_applies_patch_and_repairs_false_provenance(tmp_path):
    destination, patch = make_cached_backend(tmp_path)

    backend.prepare(tmp_path, destination)
    assert (destination / 'main.cpp').read_text() == 'patched\n'
    provenance = json.loads((destination / 'hardware_build.json').read_text())
    assert provenance['patch_sha256'] == hashlib.sha256(patch.read_bytes()).hexdigest()

    # An earlier preparer could write correct provenance without applying anything.
    (destination / 'main.cpp').write_text('old\n')
    backend.prepare(tmp_path, destination)
    assert (destination / 'main.cpp').read_text() == 'patched\n'


def test_patch_update_replaces_old_prepared_source_without_download(tmp_path, monkeypatch):
    destination, patch = make_cached_backend(tmp_path)
    backend.prepare(tmp_path, destination)
    old_provenance = json.loads((destination / 'hardware_build.json').read_text())

    def unexpected_download(*args, **kwargs):
        raise AssertionError('a verified cached archive must be reused')

    monkeypatch.setattr(backend.urllib.request, 'urlopen', unexpected_download)
    patch.write_text('--- a/main.cpp\n+++ b/main.cpp\n@@ -1 +1 @@\n-old\n+fixed\n')
    backend.prepare(tmp_path, destination)
    assert (destination / 'main.cpp').read_text() == 'fixed\n'
    provenance = json.loads((destination / 'hardware_build.json').read_text())
    assert provenance['patch_sha256'] != old_provenance['patch_sha256']
    assert provenance['patch_sha256'] == hashlib.sha256(patch.read_bytes()).hexdigest()
