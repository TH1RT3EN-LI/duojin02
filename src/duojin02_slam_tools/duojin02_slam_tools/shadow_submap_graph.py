"""Opt-in dense scan-to-rigid-submap graph; never part of default SLAM or PF.

Variables are world poses S_j and X_q. Fixed-geometry observations measure
R(S_j)^T(t(X_q)-t(S_j)) and yaw(X_q)-yaw(S_j). Caller-supplied PSD shapes define
an *experimental objective*, not calibrated information/covariance. Complete
reference ancestry is retained; overlap is reported, not declared independent.

All accepted factors are retained. Background work copies an immutable accepted
prefix, solves outside the admission lock, then commits its pose changes and
coverage atomically. New factors accepted meanwhile stay pending. An iteration
is an indivisible dense operation and may exceed a wall budget; this is neither
a hard realtime solver nor a fixed-lag marginalizer.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import hashlib
import json
import math
import threading
import time

import numpy as np

from .local_submap import RigidSubmap, ScanSubmapObservation


def _pose(value):
    result = tuple(float(v) for v in value)
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError('finite SE2 pose required')
    return result


def _hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode()).hexdigest()


def _information(value):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)) or not np.array_equal(matrix, matrix.T):
        raise ValueError('explicit symmetric finite 3x3 experimental information shape required')
    if np.linalg.eigvalsh(matrix).min() < 0:
        raise ValueError('negative experimental information shape; no flooring is applied')
    return tuple(tuple(float(v) for v in row) for row in matrix)


def residual_jacobians(submap_pose, query_pose, measurement):
    """World-additive XY/yaw perturbations; residual XY is in the submap frame.

    This is translation plus locally wrapped yaw, not an SE2 Log residual.
    The yaw derivative is local and excludes the +/-pi branch discontinuity.
    """
    sx, sy, sa = submap_pose
    qx, qy, qa = query_pose
    c, s = math.cos(sa), math.sin(sa)
    vx, vy = c*(qx-sx)+s*(qy-sy), -s*(qx-sx)+c*(qy-sy)
    residual = np.array([vx-measurement[0], vy-measurement[1],
                         math.remainder(qa-sa-measurement[2], 2*math.pi)])
    submap_jacobian = np.array([[-c, -s, vy], [s, -c, -vx], [0., 0., -1.]])
    query_jacobian = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])
    return residual, submap_jacobian, query_jacobian


@dataclass(frozen=True)
class GraphFactor:
    factor_id: str
    query_node_id: str
    query_source_id: str
    query_identity: str
    submap_id: str
    geometry_version: str
    measurement_pose_version: int
    reference_source_ids: tuple
    measurement: tuple
    experiment_information_shape: tuple
    initial_query_world_pose: tuple
    information_semantics: str = 'explicit_uncalibrated_experimental_objective'

    @property
    def fingerprint(self):
        return _hash(asdict(self))

    @property
    def evidence_key(self):
        return (self.query_source_id, self.query_identity, self.submap_id, self.geometry_version)


class ShadowSubmapGraph:
    shadow_only = True
    marginalization_implemented = False

    def __init__(self, graph_id, anchor_submap_id, parameter_scales,
                 rank_relative_tolerance=1e-10, gradient_tolerance=1e-10, step_tolerance=1e-10):
        self.parameter_scales = _pose(parameter_scales)
        if any(v <= 0 for v in self.parameter_scales) or not graph_id or not anchor_submap_id:
            raise ValueError('explicit positive physical parameter scales and graph/anchor identity required')
        if any(not math.isfinite(v) or v <= 0 for v in
               (rank_relative_tolerance, gradient_tolerance, step_tolerance)):
            raise ValueError('positive finite numerical solver tolerances required')
        self.graph_id, self.anchor_submap_id = graph_id, anchor_submap_id
        self.rank_relative_tolerance = rank_relative_tolerance
        self.gradient_tolerance, self.step_tolerance = gradient_tolerance, step_tolerance
        self._admission_lock, self._solver_lock = threading.RLock(), threading.Lock()
        self._submaps, self._queries, self._query_sources = {}, {}, {}
        self._submap_versions, self._query_versions = {}, {}
        self._physical_query_owners, self._submap_pose_history = {}, {}
        self._factors, self._evidence_owners = {}, {}
        self._ledger, self._transactions, self._failures = [], [], []
        self._covered = ()
        self._state_revision = 0
        self._last_solver = None

    def add_submap(self, submap: RigidSubmap):
        if not isinstance(submap, RigidSubmap) or not submap.scans:
            raise ValueError('a real nonempty immutable rigid submap is required')
        fingerprint = _hash(dict(id=submap.submap_id, geometry=submap.geometry_version,
                                 pose=submap.world_pose, pose_version=submap.pose_version))
        with self._admission_lock:
            if submap.submap_id in self._submaps:
                if self._submaps[submap.submap_id]['fingerprint'] != fingerprint:
                    raise ValueError('submap ID changed geometry/pose; use a new graph epoch')
                return False
            self._submaps[submap.submap_id] = dict(geometry=submap, initial_pose=submap.world_pose,
                                                  pose=submap.world_pose, fingerprint=fingerprint)
            self._submap_versions[submap.submap_id] = submap.pose_version
            self._submap_pose_history[submap.submap_id] = {submap.pose_version:submap.world_pose}
            self._ledger.append(dict(sequence=len(self._ledger), kind='submap',
                                     identity=submap.submap_id, fingerprint=fingerprint))
            return True

    def add_observation(self, factor_id, query_node_id, observation: ScanSubmapObservation,
                        initial_query_world_pose, experiment_information_shape,
                        measurement_pose_version):
        """Atomic node+factor acceptance; source support is checked before mutation."""
        if not factor_id or not query_node_id or not isinstance(observation, ScanSubmapObservation):
            raise ValueError('factor/query identity and ScanSubmapObservation required')
        initial = _pose(initial_query_world_pose)
        shape = _information(experiment_information_shape)
        measurement = _pose(observation.query_local_pose)
        references = tuple(observation.reference_source_ids)
        if not references or len(references) != len(set(references)) or (
                observation.query_source_id in references):
            raise ValueError('complete unique reference roster must exclude query source')
        if not observation.query_source_id or not observation.query_identity:
            raise ValueError('query physical source and immutable geometry identity required')
        with self._admission_lock:
            if observation.submap_id not in self._submaps:
                raise ValueError('register reference submap before observation')
            reference = self._submaps[observation.submap_id]['geometry']
            if (observation.submap_id != reference.submap_id or
                    observation.geometry_version != reference.geometry_version or
                    references != reference.source_ids):
                raise ValueError('observation invalidated by reference geometry/source revision')
            # This is the pose version used when forming the immutable local
            # measurement, not the current optimized world anchor version.
            if measurement_pose_version not in self._submap_pose_history[reference.submap_id]:
                raise ValueError('measurement reference pose version is not a known committed snapshot')
            factor = GraphFactor(factor_id, query_node_id, observation.query_source_id,
                observation.query_identity, observation.submap_id, observation.geometry_version,
                measurement_pose_version, references, measurement, shape, initial)
            if factor_id in self._factors:
                if self._factors[factor_id] != factor:
                    raise ValueError('accepted factor ID reused with different content')
                return False
            if factor.evidence_key in self._evidence_owners:
                raise ValueError('same query/reference evidence already owned by another factor')
            source_identity = (observation.query_source_id, observation.query_identity)
            owner = self._physical_query_owners.get(observation.query_source_id)
            if owner is not None and owner != (observation.query_identity, query_node_id):
                raise ValueError('physical query source already owns another geometry/node; use a new graph epoch')
            if query_node_id in self._query_sources and self._query_sources[query_node_id] != source_identity:
                raise ValueError('query node reused for another physical/geometry source')
            if query_node_id not in self._queries:
                self._queries[query_node_id] = initial
                self._query_sources[query_node_id] = source_identity
                self._query_versions[query_node_id] = 0
                self._physical_query_owners[observation.query_source_id] = (observation.query_identity, query_node_id)
            self._factors[factor_id] = factor
            self._evidence_owners[factor.evidence_key] = factor_id
            self._ledger.append(dict(sequence=len(self._ledger), kind='factor',
                                     identity=factor_id, fingerprint=factor.fingerprint))
            return True

    def _capture(self):
        with self._admission_lock:
            if self.anchor_submap_id not in self._submaps:
                raise ValueError('the declared fixed anchor submap is absent')
            poses = {('S', key): value['pose'] for key, value in self._submaps.items()}
            poses.update({('X', key): value for key, value in self._queries.items()})
            return poses, tuple(self._factors.values()), len(self._ledger)

    def _linearize(self, poses, factors):
        variables = sorted(key for key in poses if key != ('S', self.anchor_submap_id))
        columns = {key: 3*index for index, key in enumerate(variables)}
        a = np.zeros((3*len(factors), 3*len(variables)))
        residuals = np.zeros(3*len(factors))
        for index, factor in enumerate(factors):
            residual, js, jq = residual_jacobians(poses['S', factor.submap_id],
                poses['X', factor.query_node_id], factor.measurement)
            values, vectors = np.linalg.eigh(factor.experiment_information_shape)
            # PSD was checked at admission; zero eigenvalues stay exactly zero.
            whitening = np.diag(np.sqrt(values)) @ vectors.T
            rows = slice(3*index, 3*index+3)
            residuals[rows] = whitening @ residual
            for key, jacobian in ((('S', factor.submap_id), js), (('X', factor.query_node_id), jq)):
                if key in columns:
                    column = columns[key]
                    a[rows, column:column+3] = (whitening @ jacobian) * self.parameter_scales
        return variables, a, residuals

    @staticmethod
    def _objective(poses, factors):
        total = 0.
        for factor in factors:
            residual = residual_jacobians(poses['S', factor.submap_id],
                poses['X', factor.query_node_id], factor.measurement)[0]
            total += .5*float(residual @ np.asarray(factor.experiment_information_shape) @ residual)
        if not math.isfinite(total):
            raise FloatingPointError('nonfinite experimental graph objective')
        return total

    def _iteration(self, poses, factors):
        variables, a, residual = self._linearize(poses, factors)
        u, singular, vt = np.linalg.svd(a, full_matrices=True)
        threshold = self.rank_relative_tolerance * (singular[0] if len(singular) else 0.)
        rank = int(np.count_nonzero(singular > threshold))
        nullity = a.shape[1]-rank
        gradient = a.T @ residual
        info = dict(objective=.5*float(residual@residual), rank=rank, nullity=nullity,
                    singular_values=singular.tolist(), parameter_scales=self.parameter_scales,
                    nullspace_scaled=vt[rank:].tolist(),
                    rank_relative_tolerance=self.rank_relative_tolerance,
                    covariance_calibrated=False, diagonal_regularization=0.,
                    variable_order=variables, diagnostic_linearization='iteration_input_poses')
        if not variables or np.max(np.abs(gradient), initial=0.) <= self.gradient_tolerance:
            info['status'] = 'stationary_rank_deficient' if nullity else 'stationary'
            return poses, info, True
        if rank == 0:
            info['status'] = 'no_observed_direction'
            return poses, info, True
        delta = -(vt[:rank].T @ ((u[:, :rank].T @ residual)/singular[:rank]))
        if np.max(np.abs(delta), initial=0.) <= self.step_tolerance:
            info['status'] = 'small_step_rank_deficient' if nullity else 'small_step'
            return poses, info, True
        for shrink in range(21):
            scale = 2.**(-shrink)
            candidate = dict(poses)
            for index, key in enumerate(variables):
                change = delta[3*index:3*index+3] * self.parameter_scales * scale
                value = np.asarray(poses[key])+change
                value[2] = math.remainder(float(value[2]), 2*math.pi)
                candidate[key] = tuple(float(v) for v in value)
            objective = self._objective(candidate, factors)
            if objective < info['objective']:
                info.update(status='accepted_step', objective_after=objective, line_search_scale=scale)
                return candidate, info, False
        info['status'] = 'line_search_stalled'
        return poses, info, True

    def run_budget(self, max_iterations, budget_seconds, transaction_kind='background'):
        """Yield between complete dense iterations; resume from last atomic commit."""
        if not isinstance(max_iterations, int) or max_iterations < 1 or not (
                math.isfinite(budget_seconds) and budget_seconds > 0):
            raise ValueError('positive iteration and wall budgets required')
        if not self._solver_lock.acquire(blocking=False):
            raise RuntimeError('one background solver already owns this graph')
        start = time.monotonic()
        try:
            poses, factors, accepted_boundary = self._capture()
            original = dict(poses)
            objective_before = self._objective(poses, factors)
            iterations, info, terminated = 0, None, False
            while iterations < max_iterations and time.monotonic()-start < budget_seconds:
                poses, info, terminated = self._iteration(poses, factors)
                iterations += 1
                if terminated:
                    break
            if iterations == 0:
                return dict(status='yielded_before_iteration', iterations=0, committed=False,
                            elapsed_seconds=time.monotonic()-start, all_accepted_retained=True)
            objective_after = self._objective(poses, factors)
            # No user callback or mutable source data occurs after this point.
            # Admission is append-only. A newer accepted factor remains pending;
            # it is never falsely claimed to have participated in this solve.
            with self._admission_lock:
                transaction = dict(transaction_id=len(self._transactions), kind=transaction_kind,
                    state_revision=self._state_revision+1, accepted_boundary=accepted_boundary,
                    included_factor_ids=tuple(f.factor_id for f in factors), iterations=iterations,
                    objective_before=objective_before, objective_after=objective_after,
                    scheduling_yield=not terminated, solver=info,
                    elapsed_seconds=time.monotonic()-start,
                    budget_quantum='one_indivisible_dense_iteration_may_exceed_wall_budget',
                    all_accepted_retained=True,
                    newer_pending_factor_ids=tuple(k for k in self._factors if k not in
                                                   {f.factor_id for f in factors}))
                result = json.loads(json.dumps(transaction, allow_nan=False))
                for (kind, key), value in poses.items():
                    if value == original[kind, key]:
                        continue
                    if kind == 'S':
                        self._submaps[key]['pose'] = value
                        self._submap_versions[key] += 1
                        self._submap_pose_history[key][self._submap_versions[key]] = value
                    else:
                        self._queries[key] = value
                        self._query_versions[key] += 1
                self._state_revision += 1
                self._covered = tuple(f.factor_id for f in factors)
                self._transactions.append(transaction)
                self._last_solver = info
                return result
        except Exception as error:
            with self._admission_lock:
                self._failures.append(dict(kind=transaction_kind, error=type(error).__name__,
                    message=str(error), elapsed_seconds=time.monotonic()-start))
            raise
        finally:
            self._solver_lock.release()

    def finalize(self, max_iterations, budget_seconds):
        """Optimize the same in-memory ledger; never reload, drop or silently anchor nodes."""
        return self.run_budget(max_iterations, budget_seconds, transaction_kind='finalize')

    def snapshot(self):
        """An independent immutable-value snapshot; no solve or live state references."""
        with self._admission_lock:
            source_factors = {}
            for factor in self._factors.values():
                for source in set(factor.reference_source_ids + (factor.query_source_id,)):
                    source_factors.setdefault(source, []).append(factor.factor_id)
            adjacency = {('S', key):set() for key in self._submaps}
            adjacency.update({('X', key):set() for key in self._queries})
            for factor in self._factors.values():
                a, b = ('S', factor.submap_id), ('X', factor.query_node_id)
                adjacency[a].add(b); adjacency[b].add(a)
            remaining, components = set(adjacency), []
            while remaining:
                pending, group = [min(remaining)], set()
                while pending:
                    key = pending.pop()
                    if key in group:
                        continue
                    group.add(key)
                    pending.extend(adjacency[key]-group)
                remaining.difference_update(group)
                components.append(dict(nodes=sorted(group),
                    contains_fixed_anchor=('S', self.anchor_submap_id) in group))
            result = dict(schema='shadow_scan_submap_graph.v1' , graph_id=self.graph_id,
                shadow_only=True, production_pf_connected=False, global_slam_replaced=False,
                marginalization_implemented=False,
                objective_semantics='sum_explicit_uncalibrated_shapes_not_joint_physical_posterior',
                anchor_submap_id=self.anchor_submap_id, state_revision=self._state_revision,
                submaps={key:dict(world_pose=value['pose'], geometry_version=value['geometry'].geometry_version,
                    source_ids=value['geometry'].source_ids, initial_pose_version=value['geometry'].pose_version,
                    pose_version=self._submap_versions[key],
                    pose_history=[dict(pose_version=version, world_pose=pose) for version,pose in
                                  self._submap_pose_history[key].items()]) for key,value in self._submaps.items()},
                query_source_owners={key:dict(query_identity=value[0], query_node_id=value[1])
                                     for key,value in self._physical_query_owners.items()},
                queries={key:dict(world_pose=value, source_id=self._query_sources[key][0],
                    query_identity=self._query_sources[key][1], pose_version=self._query_versions[key])
                    for key,value in self._queries.items()},
                factors=[asdict(factor) for factor in self._factors.values()],
                accepted_ledger=self._ledger, accepted_factor_count=len(self._factors),
                included_factor_ids=self._covered,
                pending_factor_ids=tuple(k for k in self._factors if k not in self._covered),
                source_overlap={source:ids for source,ids in source_factors.items() if len(ids)>1},
                overlap_model='ancestry_only_no_invented_independence_or_correlation_coefficient',
                connected_components=components,
                disconnected_component_count=sum(not c['contains_fixed_anchor'] for c in components),
                last_solver=self._last_solver, transactions=self._transactions, failures=self._failures)
            return json.loads(json.dumps(result, allow_nan=False))
