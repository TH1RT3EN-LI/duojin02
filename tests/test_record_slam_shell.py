"""Execute the recording shell with controlled CLI children and real signals."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]


def fixture(tmp_path):
    hardware = tmp_path / 'hardware'
    (hardware / 'scripts').mkdir(parents=True)
    binary = hardware / 'bin'
    binary.mkdir()
    script = hardware / 'scripts/record_slam.sh'
    script.write_bytes((ROOT / 'scripts/record_slam.sh').read_bytes())
    (hardware / 'source.sh').write_text('export PATH="$RECORD_TEST_BIN:$PATH"\n')
    # Only the backend-provenance collection uses this command; ROS is mocked
    # with the real Python interpreter named explicitly in its shebang.
    python = binary / 'python3'
    python.write_text('#!/bin/bash\ncat >/dev/null\nprintf "provenance\\n" >> "$RECORD_TEST_LOG"\nprintf "{\\"fixture\\":true}\\n" > "$2"\n')
    ros = binary / 'ros2'
    ros.write_text(f'#!{sys.executable}\n' + '''import os,signal,sys,time
from pathlib import Path
args=sys.argv[1:]
with Path(os.environ['RECORD_TEST_LOG']).open('a') as stream:
    stream.write(' '.join(args)+'\\n')
if args[:2]==['param','dump']:
    print(args[2]+':\\n  ros__parameters:\\n    fixture: true')
    raise SystemExit(0)
if os.environ.get('RECORD_TEST_FAIL'):
    print('controlled recorder startup failure',file=sys.stderr)
    raise SystemExit(int(os.environ['RECORD_TEST_FAIL']))
bag=Path(args[args.index('--output')+1])
bag.mkdir()
def flush(sig,frame):
    (bag/'metadata.yaml').write_text('fixture: fully flushed\\n')
    raise SystemExit(0)
signal.signal(signal.SIGINT,flush)
signal.signal(signal.SIGTERM,flush)
(bag/'recording').write_text('ready')
while True: time.sleep(.02)
''')
    for command in (python, ros):
        command.chmod(0o755)
    calls = tmp_path / 'calls.log'
    environment = dict(os.environ, RECORD_TEST_BIN=str(binary), RECORD_TEST_LOG=str(calls))
    return script, tmp_path / 'bag', calls, environment


def test_existing_metadata_is_refused_without_any_provenance_or_query_write(tmp_path):
    script, bag, calls, environment = fixture(tmp_path)
    sidecar = Path(str(bag) + '.configuration')
    sidecar.mkdir()
    originals = {'slam-build.json': b'original backend', 'base_driver.yaml': b'original parameters'}
    for name, payload in originals.items():
        (sidecar / name).write_bytes(payload)
    result = subprocess.run(['bash', str(script), str(bag)], env=environment, capture_output=True, timeout=3)
    assert result.returncode != 0 and b'must be new' in result.stderr
    assert {path.name: path.read_bytes() for path in sidecar.iterdir()} == originals
    assert not bag.exists() and not calls.exists()


def test_existing_bag_is_refused_before_creating_metadata(tmp_path):
    script, bag, calls, environment = fixture(tmp_path)
    bag.mkdir()
    (bag / 'raw.data').write_bytes(b'original recorded data')
    result = subprocess.run(['bash', str(script), str(bag)], env=environment, capture_output=True, timeout=3)
    assert result.returncode != 0 and b'Bag directory already exists' in result.stderr
    assert (bag / 'raw.data').read_bytes() == b'original recorded data'
    assert not Path(str(bag) + '.configuration').exists() and not calls.exists()


def test_recorder_startup_failure_preserves_its_exit_code_through_cleanup(tmp_path):
    script, bag, calls, environment = fixture(tmp_path)
    environment['RECORD_TEST_FAIL'] = '7'
    result = subprocess.run(['bash', str(script), str(bag)], env=environment, capture_output=True, timeout=3)
    assert result.returncode == 7 and b'controlled recorder startup failure' in result.stderr
    assert not bag.exists()
    assert 'provenance' in calls.read_text()


def test_first_recording_creates_fresh_metadata_and_flushes_on_ctrl_c(tmp_path):
    script, bag, _, environment = fixture(tmp_path)
    process = subprocess.Popen(['bash', str(script), str(bag)], env=environment,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
    sidecar = Path(str(bag) + '.configuration')
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if (bag / 'recording').exists() and len(list(sidecar.glob('*.yaml'))) == 4:
                break
            assert process.poll() is None, 'recording stopped before the controlled signal'
            time.sleep(.02)
        assert (bag / 'recording').exists() and len(list(sidecar.glob('*.yaml'))) == 4
        os.killpg(process.pid, signal.SIGINT)
        process.communicate(timeout=3)
        assert process.returncode == 130  # Conventional shell Ctrl+C status.
        assert (bag / 'metadata.yaml').read_text() == 'fixture: fully flushed\n'
        assert (sidecar / 'slam-build.json').read_text() == '{"fixture":true}\n'
        assert not list(sidecar.glob('*.pending'))
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.communicate(timeout=3)
