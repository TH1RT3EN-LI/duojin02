#!/usr/bin/env python3
"""Prepare the project's locked NVIDIA compiler, runtime and C++ headers locally."""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import platform as host_platform
import shutil
import subprocess
import tarfile
import tempfile


COMPONENTS = ('cuda_nvcc', 'cuda_cudart', 'cuda_cccl')


def platform_name(machine=None):
    selected = machine or host_platform.machine()
    try:
        return {'x86_64': 'linux-x86_64', 'amd64': 'linux-x86_64',
                'aarch64': 'linux-aarch64', 'arm64': 'linux-aarch64'}[selected]
    except KeyError:
        raise ValueError('Unsupported CUDA toolkit architecture: ' + selected) from None


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def verify_archive(path, package):
    if Path(path).stat().st_size != package['size'] or sha256(path) != package['sha256']:
        raise ValueError('CUDA archive size or SHA256 mismatch: ' + str(path))


def obtain_archive(package, base_url, cache):
    relative = PurePosixPath(package['relative_path'])
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('Unsafe CUDA archive path in lock')
    destination = cache / relative.name
    if destination.exists():
        verify_archive(destination, package)
        return destination
    if destination.is_symlink():
        raise ValueError('CUDA cache contains a broken symlink: ' + str(destination))
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=cache, prefix='.download-', delete=False) as output:
            temporary = Path(output.name)
        subprocess.run([
            'curl', '--fail', '--location', '--retry', '3', '--retry-all-errors',
            '--connect-timeout', '30', '--max-time', '180', '--silent', '--show-error',
            '--output', str(temporary), base_url + str(relative),
        ], check=True)
        verify_archive(temporary, package)
        os.replace(temporary, destination)
        return destination
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def extract_component(archive, destination, component):
    """Strip the single archive root, retaining only links inside the toolkit."""
    expected_root = archive.name.removesuffix('.tar.xz')
    license_found = False
    with tarfile.open(archive, 'r:xz') as source:
        for member in source:
            path = PurePosixPath(member.name)
            if path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0] != expected_root:
                raise ValueError('Unsafe CUDA archive member: ' + member.name)
            relative = Path(*path.parts[1:])
            if not relative.parts:
                if not member.isdir():
                    raise ValueError('CUDA archive root must be a directory')
                continue
            if len(relative.parts) == 1 and relative.name in ('LICENSE', 'LICENSE.txt'):
                relative = Path('licenses', component, relative.name)
                license_found = member.isfile()
            target = destination / relative
            if not target.resolve().is_relative_to(destination.resolve()):
                raise ValueError('CUDA archive member escapes the toolkit: ' + member.name)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            if member.isfile():
                with source.extractfile(member) as input_stream, target.open('xb') as output:
                    shutil.copyfileobj(input_stream, output)
                target.chmod(member.mode & 0o777)
            elif member.issym():
                link = Path(member.linkname)
                if link.is_absolute() or not (target.parent / link).resolve().is_relative_to(destination.resolve()):
                    raise ValueError('CUDA archive symlink escapes the toolkit: ' + member.name)
                target.symlink_to(member.linkname)
            else:
                raise ValueError('Unsupported CUDA archive member type: ' + member.name)
    if not license_found:
        raise ValueError('CUDA component has no packaged license: ' + component)


def prepare(root, destination, cache_directory=None, selected_platform=None):
    root = Path(root).resolve()
    lock_file = root / 'docker/cuda-toolkit.json'
    lock = json.loads(lock_file.read_text())
    selected_platform = selected_platform or platform_name()
    if lock['schema_version'] != 1 or set(lock['components']) != set(COMPONENTS):
        raise ValueError('Unsupported CUDA toolkit lock schema')
    if selected_platform not in ('linux-x86_64', 'linux-aarch64'):
        raise ValueError('Unsupported CUDA toolkit platform: ' + selected_platform)
    packages = {name: lock['components'][name]['platforms'][selected_platform] for name in COMPONENTS}
    destination = Path(destination).absolute()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError('CUDA destination already exists; refusing to overwrite: ' + str(destination))
    destination.parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        staging = Path(stack.enter_context(tempfile.TemporaryDirectory(
            dir=destination.parent, prefix='.' + destination.name + '-')))
        if cache_directory is None:
            cache = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix='duojin02-cuda-downloads-')))
        else:
            cache = Path(cache_directory).resolve()
            cache.mkdir(parents=True, exist_ok=True)
        for name, package in packages.items():
            archive = obtain_archive(package, lock['base_url'], cache)
            extract_component(archive, staging, name)
        required = ('bin/nvcc', 'include/cuda_runtime.h', 'include/thrust/version.h',
                    'lib/libcudart_static.a', 'nvvm/libdevice/libdevice.10.bc')
        if not all((staging / name).is_file() for name in required):
            raise ValueError('CUDA archives did not produce a complete compiler/runtime/header toolkit')
        (staging / 'lib64').symlink_to('lib', target_is_directory=True)
        provenance = {'release': lock['release'], 'platform': selected_platform,
                      'lock_sha256': sha256(lock_file), 'manifest': lock['manifest'],
                      'components': {name: {'version': lock['components'][name]['version'],
                                           'sha256': package['sha256']} for name, package in packages.items()}}
        (staging / 'toolkit.json').write_text(json.dumps(provenance, indent=2) + '\n')
        staging.chmod(0o755)
        staging.rename(destination)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--cache-directory', type=Path)
    parser.add_argument('--platform', choices=('linux-x86_64', 'linux-aarch64'))
    args = parser.parse_args()
    try:
        destination = prepare(args.root, args.destination, args.cache_directory, args.platform)
    except (OSError, ValueError, KeyError, tarfile.TarError, subprocess.CalledProcessError) as error:
        parser.exit(1, 'CUDA toolkit preparation failed: ' + str(error) + '\n')
    print(destination)


if __name__ == '__main__':
    main()
