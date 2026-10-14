"""Locked CUDA archives are verified before atomic, local-only publication."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tarfile

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('prepare_cuda_toolkit', ROOT / 'scripts/prepare_cuda_toolkit.py')
CUDA = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CUDA)


def fixture_lock(tmp_path, unsafe=False):
    root = tmp_path / 'project'
    cache = tmp_path / 'cache'
    (root / 'docker').mkdir(parents=True)
    cache.mkdir()
    files = {
        'cuda_nvcc': {'bin/nvcc': b'compiler', 'nvvm/libdevice/libdevice.10.bc': b'bitcode'},
        'cuda_cudart': {'include/cuda_runtime.h': b'runtime', 'lib/libcudart_static.a': b'library'},
        'cuda_cccl': {'include/thrust/version.h': b'headers'},
    }
    components = {}
    for name, members in files.items():
        archive = cache / (name + '-archive.tar.xz')
        with tarfile.open(archive, 'w:xz') as stream:
            members = {**members, 'LICENSE': name.encode()}
            if unsafe and name == 'cuda_nvcc':
                members['../../escape'] = b'unsafe'
            for relative, payload in members.items():
                info = tarfile.TarInfo(name + '-archive/' + relative)
                info.size = len(payload)
                info.mode = 0o755 if relative == 'bin/nvcc' else 0o644
                stream.addfile(info, io.BytesIO(payload))
        package = {'relative_path': name + '/' + archive.name, 'sha256': CUDA.sha256(archive),
                   'size': archive.stat().st_size}
        components[name] = {'version': 'fixture', 'platforms': {
            'linux-x86_64': package, 'linux-aarch64': package}}
    lock = {'schema_version': 1, 'release': 'fixture', 'base_url': 'https://example.invalid/',
            'manifest': {'url': 'https://example.invalid/manifest.json', 'sha256': '0' * 64},
            'components': components}
    (root / 'docker/cuda-toolkit.json').write_text(json.dumps(lock))
    return root, cache, lock


def test_official_lock_covers_only_required_components_and_jetson_architecture():
    lock = json.loads((ROOT / 'docker/cuda-toolkit.json').read_text())
    assert set(lock['components']) == set(CUDA.COMPONENTS)
    assert lock['manifest']['url'] == lock['base_url'] + 'redistrib_' + lock['release'] + '.json'
    assert lock['base_url'] == 'https://developer.download.nvidia.com/compute/cuda/redist/'
    for name, component in lock['components'].items():
        assert set(component['platforms']) == {'linux-x86_64', 'linux-aarch64'}
        for platform, package in component['platforms'].items():
            assert package['relative_path'].startswith(name + '/' + platform + '/')
            assert len(package['sha256']) == 64 and package['size'] > 0


@pytest.mark.parametrize('platform', ['linux-x86_64', 'linux-aarch64'])
def test_verified_offline_archives_are_flattened_with_each_component_license(tmp_path, monkeypatch, platform):
    root, cache, _ = fixture_lock(tmp_path)
    monkeypatch.setattr(CUDA.subprocess, 'run', lambda *a, **k: pytest.fail('cache must work offline'))
    hashes = {path.name: CUDA.sha256(path) for path in cache.iterdir()}
    destination = tmp_path / 'toolkit'
    CUDA.prepare(root, destination, cache, platform)
    assert (destination / 'bin/nvcc').read_bytes() == b'compiler'
    assert (destination / 'bin/nvcc').stat().st_mode & 0o111
    assert (destination / 'lib64').is_symlink()
    assert (destination / 'lib64/libcudart_static.a').read_bytes() == b'library'
    for name in CUDA.COMPONENTS:
        assert (destination / 'licenses' / name / 'LICENSE').read_bytes() == name.encode()
    assert json.loads((destination / 'toolkit.json').read_text())['platform'] == platform
    assert {path.name: CUDA.sha256(path) for path in cache.iterdir()} == hashes


def test_hash_mismatch_does_not_publish_or_modify_cached_archive(tmp_path, monkeypatch):
    root, cache, _ = fixture_lock(tmp_path)
    archive = cache / 'cuda_nvcc-archive.tar.xz'
    archive.write_bytes(b'corrupt')
    monkeypatch.setattr(CUDA.subprocess, 'run', lambda *a, **k: pytest.fail('bad cache must not be replaced'))
    with pytest.raises(ValueError, match='SHA256 mismatch'):
        CUDA.prepare(root, tmp_path / 'toolkit', cache, 'linux-x86_64')
    assert not (tmp_path / 'toolkit').exists()
    assert archive.read_bytes() == b'corrupt'
    assert not list(tmp_path.glob('.toolkit-*'))


def test_verified_archive_cannot_escape_extraction_directory(tmp_path):
    root, cache, _ = fixture_lock(tmp_path, unsafe=True)
    with pytest.raises(ValueError, match='Unsafe CUDA archive member'):
        CUDA.prepare(root, tmp_path / 'toolkit', cache, 'linux-x86_64')
    assert not (tmp_path / 'toolkit').exists()
    assert not (tmp_path / 'escape').exists()
    assert not list(tmp_path.glob('.toolkit-*'))


def test_download_is_published_only_after_exact_hash_and_size_validation(tmp_path, monkeypatch):
    payload = b'official fixture archive'
    package = {'relative_path': 'component/archive.tar.xz', 'size': len(payload),
               'sha256': hashlib.sha256(payload).hexdigest()}
    calls = []

    def download(content):
        def transport(command, **kwargs):
            calls.append((command, kwargs))
            Path(command[-2]).write_bytes(content)
        return transport

    for corrupt in (b'bad', b'x' * len(payload)):
        monkeypatch.setattr(CUDA.subprocess, 'run', download(corrupt))
        with pytest.raises(ValueError, match='SHA256 mismatch'):
            CUDA.obtain_archive(package, 'https://example.invalid/', tmp_path)
        assert list(tmp_path.iterdir()) == []
    monkeypatch.setattr(CUDA.subprocess, 'run', download(payload))
    assert CUDA.obtain_archive(package, 'https://example.invalid/', tmp_path).read_bytes() == payload
    assert [path.name for path in tmp_path.iterdir()] == ['archive.tar.xz']
    command, kwargs = calls[-1]
    assert command[:6] == ['curl', '--fail', '--location', '--retry', '3', '--retry-all-errors']
    assert command[command.index('--max-time') + 1] == '180'
    assert command[-1] == 'https://example.invalid/component/archive.tar.xz'
    assert kwargs == {'check': True}


def test_failed_transport_removes_partial_download_and_does_not_publish_toolkit(tmp_path, monkeypatch):
    root, cache, _ = fixture_lock(tmp_path)
    (cache / 'cuda_nvcc-archive.tar.xz').unlink()

    def fail(command, **kwargs):
        Path(command[-2]).write_bytes(b'partial download')
        raise subprocess.CalledProcessError(35, command)

    monkeypatch.setattr(CUDA.subprocess, 'run', fail)
    with pytest.raises(subprocess.CalledProcessError):
        CUDA.prepare(root, tmp_path / 'toolkit', cache, 'linux-x86_64')
    assert not (tmp_path / 'toolkit').exists()
    assert not (cache / 'cuda_nvcc-archive.tar.xz').exists()
    assert not list(cache.glob('.download-*'))
    assert not list(tmp_path.glob('.toolkit-*'))


def test_unsupported_platform_and_existing_destination_are_rejected(tmp_path):
    root, cache, _ = fixture_lock(tmp_path)
    with pytest.raises(ValueError, match='architecture'):
        CUDA.platform_name('riscv64')
    with pytest.raises(ValueError, match='platform'):
        CUDA.prepare(root, tmp_path / 'toolkit', cache, 'linux-sbsa')
    destination = tmp_path / 'toolkit'
    destination.mkdir()
    (destination / 'user-file').write_text('keep')
    with pytest.raises(FileExistsError, match='refusing to overwrite'):
        CUDA.prepare(root, destination, cache, 'linux-x86_64')
    assert (destination / 'user-file').read_text() == 'keep'
