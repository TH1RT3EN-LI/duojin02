"""Independent objective/geometry audit must reject plausible incomplete event streams."""
import gzip
import importlib.util
import json
from pathlib import Path

import pytest

pytest.importorskip("numpy", reason="Graph-objective validation uses the isolated ROS runtime NumPy")

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("graph_event_validator", ROOT / "tests/integration/map_graph/validate_graph_events.py")
validator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validator)


def events():
    return [
        dict(event="session", schema=2, state_mode="delta", loss="None", loss_scale=.7,
             residual_coordinates="source_base_xy_yaw", whitening="upper_llt_left"),
        dict(event="node_added", node_id=0, pose=[0., 0., 0.], raw_stamp_ns=10,
             geometry_version="prepared-typed-rays.v2"),
        dict(event="node_added", node_id=1, pose=[2., 0., 0.], raw_stamp_ns=20,
             geometry_version="prepared-typed-rays.v2"),
        dict(event="edge_added", edge_id=1, source_id=0, target_id=1,
             measurement_base_i=[1., 0., 0.], covariance_base_i_row_major=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
             provenance="sequential"),
        dict(event="optimization_begin", optimization_id=1, anchor_id=0, node_count=2,
             edge_count=1, changed_nodes=[]),
        dict(event="optimization_end", optimization_id=1, anchor_id=0, node_count=2,
             edge_count=1, changed_nodes=[[1, 1., 0., 0.]], initial_cost=.5, final_cost=0.,
             usable=True, successful_steps=1, total_time_seconds=.001),
    ]


def write(path, rows):
    with gzip.open(path, "wt") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")


def test_lossless_delta_stream_reconstructs_both_actual_objectives(tmp_path):
    path = tmp_path / "graph.jsonl.gz"
    write(path, events())
    result = validator.validate(path)
    assert result["passed"] and result["optimization_count"] == 1
    assert result["independent_edge_residual_checks"] == 2
    assert result["maximum_reconstructed_objective_absolute_error"] == 0


@pytest.mark.parametrize("mutation,expected", [
    (lambda rows: rows[-1].update(changed_nodes=[]), "final objective cannot be reconstructed"),
    (lambda rows: rows[-1].update(edge_count=2), "Live edge reconstruction is incomplete"),
    (lambda rows: rows[-1].update(changed_nodes=[[0, .1, 0., 0.], [1, 1.1, 0., 0.]]), "fixed gauge anchor"),
])
def test_incomplete_or_wrong_node_state_is_not_accepted(tmp_path, mutation, expected):
    rows = events()
    mutation(rows)
    path = tmp_path / "graph.jsonl.gz"
    write(path, rows)
    with pytest.raises(AssertionError, match=expected):
        validator.validate(path)


def test_coarse_fine_clone_geometry_mismatch_is_rejected(tmp_path):
    rows = events()
    detail = dict(stage="coarse", provenance="explicit_loop", query_id=1, query_raw_stamp_ns=20,
                  query_reference_stamp_ns=25, query_geometry_version="prepared-typed-rays.v2",
                  chain_ids=[0], decision="accepted", reason="coarse_gate_passed")
    rows.insert(4, dict(event="mapper_match", match_id=1, detail=detail))
    fine = dict(detail, stage="fine", query_geometry_version="legacy-raw.v0", reason="fine_gate_passed")
    rows.insert(5, dict(event="mapper_match", match_id=2, detail=fine))
    path = tmp_path / "graph.jsonl.gz"
    write(path, rows)
    with pytest.raises(AssertionError, match="geometry changed across consumer paths"):
        validator.validate(path)
