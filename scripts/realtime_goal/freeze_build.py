#!/usr/bin/env python3
"""Bind immutable build inputs and private installed artifacts after compilation."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def manifest(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--install', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists(): p.error('manifest exists; use a new attempt path')
    result = {'source_root': str(a.source), 'install_root': str(a.install),
              'source_sha256': manifest(a.source), 'installed_sha256': manifest(a.install),
              'git_head_context_only': subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
              'binding': 'source copied before build; this manifest frozen immediately after build; actual runtime loader checked separately'}
    with a.output.open('x') as out: json.dump(result, out, indent=2);out.write('\n')


if __name__ == '__main__': main()
