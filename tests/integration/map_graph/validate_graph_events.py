#!/usr/bin/env python3
"""Stream-validate production Ceres JSONL (including gzip/delta) by reconstructing its graph.

Every actual solve's objective is independently recomputed from edge definitions and
exact before/after node states; no final solve is inferred from map serialization.
"""
import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def reject_constant(value):
    raise ValueError(f"Non-JSON numeric literal: {value}")


def validate(path, require_solves=True):
    path = Path(path)
    nodes, node_metadata, edges, pairs, beginnings, pending_fine = {}, {}, {}, {}, {}, {}
    event_counts, decisions, provenances, geometry_versions = Counter(), Counter(), Counter(), Counter()
    optimization_summaries = []
    session_count = residual_checks = full_norm_checks = changed_node_rows = uncompressed_bytes = 0
    max_cost_error = 0.0
    loss, scale = None, None
    last_match_id = 0
    digest = hashlib.sha256()

    def objective():
        nonlocal residual_checks
        if not edges:
            return 0.0, {}
        ordered = list(edges.values())
        source = np.array([nodes[edge["source_id"]] for edge in ordered])
        target = np.array([nodes[edge["target_id"]] for edge in ordered])
        measurement = np.array([edge["measurement_base_i"] for edge in ordered])
        delta = target[:, :2] - source[:, :2]
        cosine, sine = np.cos(source[:, 2]), np.sin(source[:, 2])
        residual = np.column_stack((cosine * delta[:, 0] + sine * delta[:, 1],
                                    -sine * delta[:, 0] + cosine * delta[:, 1],
                                    target[:, 2] - source[:, 2])) - measurement
        residual[:, 2] -= 2.0 * math.pi * np.floor((residual[:, 2] + math.pi) / (2.0 * math.pi))
        precision = np.array([edge["_precision"] for edge in ordered])
        squared = np.einsum("ni,nij,nj->n", residual, precision, residual)
        require(np.all(np.isfinite(squared)) and np.min(squared) >= -1e-10,
                "Invalid independently reconstructed edge residual")
        squared = np.maximum(squared, 0.0)
        if loss == "None":
            rho = squared
        elif loss == "HuberLoss":
            rho = np.where(squared <= scale * scale, squared, 2.0 * scale * np.sqrt(squared) - scale * scale)
        elif loss == "CauchyLoss":
            rho = scale * scale * np.log1p(squared / (scale * scale))
        else:
            raise AssertionError(f"Unknown recorded robust loss: {loss}")
        residual_checks += len(ordered)
        return float(np.sum(rho) / 2.0), {edge["edge_id"]: float(norm) for edge, norm in zip(ordered, squared)}

    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rb") as stream:
        for line_number, blob in enumerate(stream, 1):
            digest.update(blob)
            uncompressed_bytes += len(blob)
            require(blob.endswith(b"\n"), f"Truncated event at line {line_number}")
            event = json.loads(blob, parse_constant=reject_constant)
            kind = event["event"]
            event_counts[kind] += 1
            if kind == "session":
                require(not beginnings, "New session interrupts an actual optimization")
                nodes, node_metadata, edges, pairs, pending_fine = {}, {}, {}, {}, {}
                loss, scale = event["loss"], event["loss_scale"]
                require(event["residual_coordinates"] == "source_base_xy_yaw", "Unrecognized residual coordinates")
                require(event["whitening"] == "upper_llt_left", "Unrecognized whitening")
                session_count += 1
                last_match_id = 0
            elif kind in {"node_added", "node_modified"}:
                node_id = event["node_id"]
                pose = np.array(event["pose"], dtype=float)
                require(pose.shape == (3,) and np.all(np.isfinite(pose)), "Nonfinite node event")
                nodes[node_id] = pose
                if kind == "node_added":
                    node_metadata[node_id] = event
                    geometry_versions[event.get("geometry_version", "unknown")] += 1
            elif kind == "node_removed":
                node_id = event["node_id"]
                require(all(node_id not in (edge["source_id"], edge["target_id"]) for edge in edges.values()),
                        "Node removed while its edges remain active")
                del nodes[node_id]
                node_metadata.pop(node_id, None)
            elif kind == "edge_added":
                edge_id = event["edge_id"]
                require(edge_id not in edges, "Reused live edge ID")
                pair = (event["source_id"], event["target_id"])
                require(pair not in pairs, "Duplicate directed graph edge was silently accepted")
                require(all(node_id in nodes for node_id in pair), "Edge refers to an absent node")
                covariance = np.array(event["covariance_base_i_row_major"]).reshape(3, 3)
                require(np.all(np.isfinite(covariance)) and np.allclose(covariance, covariance.T, atol=1e-12),
                        "Nonfinite/asymmetric edge covariance")
                np.linalg.cholesky(covariance)
                event["_precision"] = np.linalg.inv(covariance)
                edges[edge_id], pairs[pair] = event, edge_id
                provenances[event["provenance"]] += 1
            elif kind == "edge_removed":
                edge = edges.pop(event["edge_id"])
                del pairs[(edge["source_id"], edge["target_id"])]
            elif kind == "reset":
                require(not beginnings, "Reset interrupts an optimization")
                nodes, node_metadata, edges, pairs, pending_fine = {}, {}, {}, {}, {}
            elif kind == "mapper_match":
                require(event["match_id"] == last_match_id + 1, "Mapper event ID discontinuity")
                last_match_id = event["match_id"]
                detail = event["detail"]
                decisions[f'{detail["provenance"]}/{detail["stage"]}/{detail["decision"]}/{detail["reason"]}'] += 1
                require(detail["decision"] in {"added", "duplicate", "accepted", "rejected"}, "Unknown match decision")
                query_id = detail["query_id"]
                if query_id in node_metadata:
                    metadata = node_metadata[query_id]
                    require(detail["query_raw_stamp_ns"] == metadata["raw_stamp_ns"], "Query source scan changed across consumer paths")
                    require(detail["query_geometry_version"] == metadata["geometry_version"], "Query geometry changed across consumer paths")
                if detail["provenance"] == "explicit_loop":
                    key = (query_id, tuple(detail["chain_ids"]))
                    if detail["stage"] == "coarse" and detail["decision"] == "accepted":
                        pending_fine[key] = detail
                    elif detail["stage"] == "fine":
                        previous = pending_fine.pop(key)
                        require(previous["query_geometry_version"] == detail["query_geometry_version"],
                                "Explicit-loop coarse/fine geometries disagree")
                        require(previous["query_raw_stamp_ns"] == detail["query_raw_stamp_ns"] and
                                previous["query_reference_stamp_ns"] == detail["query_reference_stamp_ns"],
                                "Explicit-loop coarse/fine time provenance disagrees")
            elif kind in {"optimization_begin", "optimization_end"}:
                for row in event.get("nodes", event.get("changed_nodes", [])):
                    pose = np.array(row[1:], dtype=float)
                    require(pose.shape == (3,) and np.all(np.isfinite(pose)), "Nonfinite optimized node")
                    nodes[row[0]] = pose
                    changed_node_rows += 1
                require(len(nodes) == event.get("node_count", len(nodes)), "Node state reconstruction is incomplete")
                require(len(edges) == event.get("edge_count", len(edges)), "Live edge reconstruction is incomplete")
                cost, squared = objective()
                for item in event.get("edges", []):
                    require(math.isclose(squared[item["edge_id"]], item["whitened_squared_norm"], rel_tol=1e-9, abs_tol=1e-9),
                            "Recorded residual differs from reconstructed residual")
                    full_norm_checks += 1
                optimization_id = event["optimization_id"]
                if kind == "optimization_begin":
                    require(optimization_id not in beginnings, "Duplicate optimization begin")
                    anchor = event["anchor_id"]
                    beginnings[optimization_id] = (cost, anchor, None if anchor is None else nodes[anchor].copy())
                else:
                    initial_cost, anchor, initial_anchor = beginnings.pop(optimization_id)
                    errors = (abs(initial_cost - event["initial_cost"]), abs(cost - event["final_cost"]))
                    max_cost_error = max(max_cost_error, *errors)
                    require(math.isclose(initial_cost, event["initial_cost"], rel_tol=2e-7, abs_tol=1e-7),
                            f"Optimization {optimization_id} initial objective cannot be reconstructed: {initial_cost} vs {event['initial_cost']}")
                    require(math.isclose(cost, event["final_cost"], rel_tol=2e-7, abs_tol=1e-7),
                            f"Optimization {optimization_id} final objective cannot be reconstructed: {cost} vs {event['final_cost']}")
                    if anchor is not None:
                        require(np.array_equal(initial_anchor, nodes[anchor]), "Optimizer moved fixed gauge anchor")
                    optimization_summaries.append(dict(optimization_id=optimization_id, node_count=len(nodes), edge_count=len(edges),
                                                       initial_cost=initial_cost, final_cost=cost, usable=event["usable"],
                                                       successful_steps=event["successful_steps"], total_time_seconds=event["total_time_seconds"]))
            else:
                raise AssertionError(f"Unknown event kind at line {line_number}: {kind}")
    require(session_count > 0, "Missing actual Ceres session")
    require(not beginnings, "Incomplete actual optimization begin/end pair")
    require(not pending_fine, "Accepted coarse match lacks actual fine event")
    require(not require_solves or len(optimization_summaries) > 0, "No actual optimization was recorded")
    archive_checks = {}
    node_path, edge_path = path.parent/"graph_nodes.csv", path.parent/"graph_edges.csv"
    if node_path.exists() and edge_path.exists():
        archived_nodes = {int(row["node_id"]): row for row in csv.DictReader(node_path.open())}
        require(set(archived_nodes) == set(nodes), "Saved archive node set differs from actual event graph")
        for node_id, row in archived_nodes.items():
            metadata = node_metadata[node_id]
            require(int(row["raw_stamp_ns"]) == metadata["raw_stamp_ns"] and
                    int(row["reference_stamp_ns"]) == metadata["reference_stamp_ns"] and
                    row["geometry_version"] == metadata["geometry_version"],
                    "Saved archive node measurement identity differs from actual events")
        archived_edges = {(int(row["source_id"]), int(row["target_id"])): row for row in csv.DictReader(edge_path.open())}
        require(set(archived_edges) == set(pairs), "Saved archive edge set differs from actual event graph")
        for pair, row in archived_edges.items():
            edge = edges[pairs[pair]]
            require(row["provenance"] == edge["provenance"], "Saved archive edge provenance differs from actual events")
            require(np.array_equal([float(row[f"measurement_{axis}"]) for axis in ("x", "y", "yaw")], edge["measurement_base_i"]),
                    "Saved archive edge measurement differs from actual events")
            require(np.array_equal([float(row[f"cov_{r}{c}"]) for r in range(3) for c in range(3)], edge["covariance_base_i_row_major"]),
                    "Saved archive edge covariance differs from actual events")
        archive_checks = dict(saved_archive_node_identity_matches_events=True, saved_archive_edge_measurements_match_events=True,
                              archive_node_count=len(archived_nodes), archive_edge_count=len(archived_edges))
    return dict(passed=True, archive_checks=archive_checks, schema="reconstructed-production-graph-events.v1", source=str(path),
                uncompressed_sha256=digest.hexdigest(), compressed_bytes=path.stat().st_size,
                uncompressed_bytes=uncompressed_bytes, sessions=session_count,
                event_counts=dict(event_counts), match_decisions=dict(decisions),
                edge_provenance_counts=dict(provenances), node_geometry_version_counts=dict(geometry_versions),
                optimization_count=len(optimization_summaries), independent_edge_residual_checks=residual_checks,
                full_logged_norm_checks=full_norm_checks, changed_node_rows=changed_node_rows,
                maximum_reconstructed_objective_absolute_error=max_cost_error,
                final_live_node_count=len(nodes), final_live_edge_count=len(edges),
                map_save_semantics="serialize_current_graph; no implicit final all-edge optimization",
                optimizations=optimization_summaries)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-no-solves", action="store_true")
    args = parser.parse_args()
    result = validate(args.source, require_solves=not args.allow_no_solves)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({key: value for key, value in result.items() if key != "optimizations"}, indent=2))
