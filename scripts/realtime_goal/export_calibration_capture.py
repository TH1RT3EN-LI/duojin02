#!/usr/bin/env python3
"""Index E3 shadow residuals for independent repeat/surface calibration.

No noise parameters are fitted: these residuals used fitted poses/associations,
so their empirical spread is not independent physical error calibration. D0/D1
uses of the same query beam keep the same physical sample key.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--residuals',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    blob=a.residuals.read_bytes(); rows=[json.loads(s) for s in blob.splitlines() if s.strip()]
    keys=collections.Counter((r['query_source_id'],r['query_beam_index']) for r in rows)
    groups=collections.Counter((r['geometry_version'],r['surface_source_id']) for r in rows)
    result={'schema':'shared_error_calibration_capture.v1','input':str(a.residuals),
            'input_sha256':hashlib.sha256(blob).hexdigest(),'rows':len(rows),
            'distinct_physical_query_samples':len(keys),
            'reused_query_samples_across_shadow_methods':sum(n>1 for n in keys.values()),
            'groups':[{'geometry_version':g,'reference_source':s,'rows':n}
                      for (g,s),n in sorted(groups.items())],
            'physical_sample_key':['query_source_id','query_beam_index'],
            'calibration_status':'unvalidated','likelihood_weight_exported':False,
            'residual_selection':'conditional fitted pose and association; development capture only',
            'missing_independent_evidence':['independent repeated frozen-map queries',
                'external or independent surface/map reference',
                'sensor time/extrinsic excitation and calibration record',
                'held-out residual mean/coverage tests after parameter freeze'],
            'cross_scan_lifetime':'same surface/map latent persists across all source uses; never reset per frame'}
    with a.output.open('x') as f: json.dump(result,f,indent=2);f.write('\n')


if __name__=='__main__':main()
