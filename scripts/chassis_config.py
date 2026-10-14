#!/usr/bin/env python3
"""Validate, admit, or export the sole chassis profile; never start devices."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/duojin01_bringup'))
from duojin01_bringup.chassis_profile import (
    COMPONENTS, ENVIRONMENT_KEYS, PROFILE_RELATIVE_PATH, SCHEMA_VERSION,
    load_profile, payload, render, simulation_profile,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['check', 'adopt', 'env', 'export'])
    parser.add_argument('--profile', type=Path, default=ROOT / PROFILE_RELATIVE_PATH)
    parser.add_argument('--component', choices=sorted(COMPONENTS | {'lidar', 'slam_offline', 'simulation'}))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = args.profile.resolve(strict=True)
    profile = load_profile(source)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if args.command == 'env':
        for key in sorted(ENVIRONMENT_KEYS):
            print(key + '=' + profile['runtime_environment'][key])
        return
    if args.command == 'export':
        if not args.component:
            parser.error('export requires --component')
        if args.component == 'simulation':
            import yaml
            data = yaml.safe_dump(simulation_profile(source), sort_keys=False).encode()
        else:
            data = payload(profile, args.component)
        if args.output:
            args.output.write_bytes(data)
        else:
            sys.stdout.buffer.write(data)
        return
    hashes = {name: hashlib.sha256(payload(profile, name)).hexdigest()
              for name in sorted(COMPONENTS | {'lidar', 'slam_offline'})}
    result = dict(ok=True, schema_version=SCHEMA_VERSION, profile=str(source),
                  profile_sha256=digest, motion=profile['motion'], derived_component_sha256=hashes)
    if args.command == 'adopt':
        if source != (ROOT / PROFILE_RELATIVE_PATH).resolve():
            parser.error('production admission requires this workspace\'s sole chassis.yaml')
        manifest_path = ROOT / 'docs/evidence/orin-top1/release.json'
        manifest = json.loads(manifest_path.read_text())
        previous = dict(manifest['profile'])
        required = dict(manifest['required_install_files'])
        manifest['configuration_schema'] = SCHEMA_VERSION
        manifest['profile'] = dict(path=PROFILE_RELATIVE_PATH, sha256=digest,
                                   derived_component_sha256=hashes)
        manifest['runtime_environment'] = dict(profile['runtime_environment'])
        manifest['configuration_revision'] = dict(
            change='Single chassis authority; deterministic runtime-only ROS/simulator derivatives',
            previous_profile=previous,
            validation_scope='Configuration and native artifact admission; no new motion or timing claim')
        assert manifest['required_install_files'] == required
        data = (json.dumps(manifest, indent=2) + '\n').encode()
        with tempfile.NamedTemporaryFile(dir=manifest_path.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, manifest_path)
        result['release_updated'] = True
        result['native_artifact_pins_changed'] = False
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, TypeError) as error:
        print(json.dumps(dict(ok=False, error=str(error)), ensure_ascii=False), file=sys.stderr)
        raise SystemExit(1)
