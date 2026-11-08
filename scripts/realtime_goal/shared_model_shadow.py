#!/usr/bin/env python3
"""Evaluate explicit, provenance-bearing E4 experiment inputs, never PF weights.

Input JSON declares state/geometry/linearization/physical shared variables,
shared_covariance and observation blocks with y/J/B/R. Covariances have NO
fallback defaults. Use independently collected/calibrated inputs for physical
interpretation; synthetic data remain synthetic even when this operator passes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/duojin02_slam_tools'))
from duojin02_slam_tools.shared_evidence import EvidenceBlock, PersistentSharedGaussian, SharedVariable


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error('output already exists; choose a new attempt path')
    blob = a.input.read_bytes(); data = json.loads(blob)
    m = PersistentSharedGaussian(data['state_dimension'],
        [SharedVariable(**v) for v in data['variables']], data['shared_covariance'],
        data['geometry_identity'], data['linearization_identity'], data['max_observations'])
    records = []
    for block in data['blocks']:
        accepted = m.append(EvidenceBlock(**block))
        records.append({'evidence_id': block['evidence_id'], 'new_physical_evidence': accepted,
                        'nll': m.nll(data['evaluation_state']),
                        'conditional_increment_nll': m.last_conditional_nll(data['evaluation_state']) if accepted else 0.})
    result = {'input_sha256': hashlib.sha256(blob).hexdigest(), 'audit': m.audit_record(),
              'records': records, 'physical_mean_or_coverage_benefit': 'unmeasured',
              'covariance_assumption_provenance': data['covariance_assumption_provenance']}
    with a.output.open('x') as out:
        json.dump(result, out, indent=2, allow_nan=False); out.write('\n')


if __name__ == '__main__': main()
