#!/usr/bin/env python3
"""Validate the native production Ceres event stream against coordinate residuals."""
import json
import math
import sys
from pathlib import Path
import numpy as np


def validate(path):
    edges, beginning = {}, {}
    events = solves = norms = 0
    loss, scale = None, None
    for line in path.read_text().splitlines():
        event = json.loads(line, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        kind = event['event']
        events += 1
        if kind == 'session':
            loss, scale = event['loss'], event['loss_scale']
            edges, beginning = {}, {}
            assert event['residual_coordinates'] == 'source_base_xy_yaw'
            assert event['whitening'] == 'upper_llt_left'
        elif kind == 'edge_added':
            assert event['provenance'] in {'sequential', 'running_chain', 'explicit_loop'}
            assert event['chain_ids'] == [0, 1, 2]
            assert event['search_boundary_hit'] is None
            edges[event['edge_id']] = event
        elif kind == 'edge_removed':
            del edges[event['edge_id']]
        elif kind == 'reset':
            edges = {}
        elif kind.startswith('optimization_'):
            nodes = {row[0]: np.array(row[1:]) for row in event['nodes']}
            cost = 0.
            for item in event['edges']:
                edge = edges[item['edge_id']]
                a, b = nodes[edge['source_id']], nodes[edge['target_id']]
                delta = b[:2] - a[:2]
                c, s = math.cos(a[2]), math.sin(a[2])
                residual = np.array([c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1], math.remainder(b[2]-a[2], 2*math.pi)]) - edge['measurement_base_i']
                residual[2] = math.remainder(residual[2], 2*math.pi)
                covariance = np.array(edge['covariance_base_i_row_major']).reshape(3, 3)
                squared = float(residual @ np.linalg.solve(covariance, residual))
                assert math.isclose(squared, item['whitened_squared_norm'], rel_tol=1e-11, abs_tol=1e-11)
                derivative = (1/(1+squared/(scale*scale)) if loss == 'CauchyLoss' else
                              1. if loss == 'None' or squared <= scale*scale else scale/math.sqrt(squared))
                assert math.isclose(derivative, item['rho_prime'], rel_tol=1e-11, abs_tol=1e-11)
                cost += item['robust_cost']
                norms += 1
            if kind == 'optimization_begin':
                beginning[event['optimization_id']] = (cost, nodes)
            else:
                initial_cost, initial_nodes = beginning.pop(event['optimization_id'])
                assert math.isclose(event['initial_cost'], initial_cost, rel_tol=1e-10, abs_tol=1e-10)
                assert math.isclose(event['final_cost'], cost, rel_tol=1e-10, abs_tol=1e-10)
                assert event['usable']
                if event['anchor_id'] is not None:
                    assert np.array_equal(initial_nodes[event['anchor_id']], nodes[event['anchor_id']])
                solves += 1
    assert not beginning
    assert solves > 0
    return {'events': events, 'solves': solves, 'independent_whitened_norm_checks': norms}


if __name__ == '__main__':
    result = {name: validate(Path(sys.argv[1]) / f'{name}.jsonl') for name in ['none', 'huber', 'cauchy']}
    print(json.dumps(result, indent=2))
