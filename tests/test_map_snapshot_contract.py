"""Snapshot integrity must describe the actual artifacts each consumer reads."""

import json
from pathlib import Path
import sys
import struct
import zlib

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/duojin02_slam_tools'))
from duojin02_slam_tools.snapshots import artifact_path, commit_snapshot, reserve_snapshot, validate_map


def png_pixel():
    def chunk(kind, payload):
        return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', zlib.crc32(kind + payload))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 1, 1, 8, 0, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(b'\x00\xfe')) + chunk(b'IEND', b''))


def snapshot(directory, name):
    with reserve_snapshot(directory, name) as (pending, final):
        artifact_path(pending, '.yaml').write_text(
            f'image: {name}.pgm\nresolution: 0.02\norigin: [0, 0, 0]\n')
        artifact_path(pending, '.pgm').write_bytes(b'P5\n1 1\n255\n\xfe')
        artifact_path(pending, '.posegraph').write_bytes(b'graph')
        artifact_path(pending, '.data').write_bytes(b'scans')
        return commit_snapshot(pending, final, {'fixture': True})


def test_foreign_manifest_cannot_verify_another_selected_map(tmp_path):
    first, second = snapshot(tmp_path, 'first'), snapshot(tmp_path, 'second')
    second.with_suffix('.snapshot.json').write_bytes(first.with_suffix('.snapshot.json').read_bytes())
    second.with_suffix('.pgm').write_bytes(b'changed unverified raster')
    with pytest.raises(ValueError, match='does not match the selected'):
        validate_map(second)


def test_image_role_must_bind_to_the_image_referenced_by_the_yaml(tmp_path):
    selected = snapshot(tmp_path, 'selected')
    other = snapshot(tmp_path, 'other')
    manifest_file = selected.with_suffix('.snapshot.json')
    manifest = json.loads(manifest_file.read_text())
    manifest['files']['image'] = json.loads(other.with_suffix('.snapshot.json').read_text())['files']['image']
    manifest_file.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='role image does not match'):
        validate_map(selected)


@pytest.mark.parametrize('files', [{}, [], None, {'yaml': {}}, {'yaml': None}])
def test_declared_snapshot_cannot_omit_required_file_roles(tmp_path, files):
    selected = snapshot(tmp_path, 'selected')
    selected.with_suffix('.snapshot.json').write_text(json.dumps({'schema': 1, 'files': files}))
    with pytest.raises(ValueError, match='snapshot'):
        validate_map(selected)


@pytest.mark.parametrize('artifact', ['.posegraph', '.data'])
def test_raster_consumer_ignores_unused_graph_damage_but_continuation_rejects_it(tmp_path, artifact):
    selected = snapshot(tmp_path, 'selected')
    selected.with_suffix(artifact).write_bytes(b'damaged graph')
    validate_map(selected)
    with pytest.raises(ValueError, match='modified'):
        validate_map(selected, require_graph=True)


@pytest.mark.parametrize('role', ['posegraph', 'data'])
def test_declared_continuation_snapshot_requires_both_graph_roles(tmp_path, role):
    selected = snapshot(tmp_path, 'selected')
    manifest_file = selected.with_suffix('.snapshot.json')
    manifest = json.loads(manifest_file.read_text())
    del manifest['files'][role]
    manifest_file.write_text(json.dumps(manifest))
    validate_map(selected)
    with pytest.raises(ValueError, match='Missing or invalid snapshot role'):
        validate_map(selected, require_graph=True)


@pytest.mark.parametrize('extension,payload', [
    ('pgm', b'P5\n1 1\n255\n\xfe'), ('pnm', b'P2\n1 1\n255\n254\n'),
    ('png', png_pixel()),
])
def test_legacy_maps_without_manifest_keep_supported_image_formats(tmp_path, extension, payload):
    image = tmp_path / ('legacy.' + extension)
    image.write_bytes(payload)
    selected = tmp_path / 'legacy.yaml'
    selected.write_text(f'image: {image.name}\nresolution: 0.02\norigin: [0, 0, 0]\n')
    assert validate_map(selected)[1] == image


@pytest.mark.parametrize('geometry', ['resolution: true\norigin: [0, 0, 0]',
                                      'resolution: 0.02\norigin: [false, 0, 0]'])
def test_boolean_values_are_not_real_map_geometry(tmp_path, geometry):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\xfe')
    selected = tmp_path / 'map.yaml'
    selected.write_text(f'image: map.pgm\n{geometry}\n')
    with pytest.raises(ValueError, match='Invalid map'):
        validate_map(selected)
