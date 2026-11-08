"""Experimental joint Gaussian evidence model; never called by the production PF.

For a *fixed declared linearization*, y = J x + B eta + epsilon.  A persistent
eta belongs to a physical source/geometry, and is integrated out ONCE over the
entire accumulated block. Incremental NLL is joint(new+old)-joint(old), i.e. the
conditional likelihood, not a product of separately marginalized scans.

This implementation is a bounded offline/shadow operator, not a fixed-lag
filter: it refuses overflow and never silently drops historical observations.
No covariance here is a calibrated robot/goal accuracy claim.
"""
from dataclasses import dataclass
import hashlib
import json
from typing import Tuple

import numpy as np


def _array(value, ndim, name):
    result = np.array(value, dtype=float, copy=True)
    if result.ndim != ndim or not np.all(np.isfinite(result)):
        raise ValueError(f"{name}: expected finite rank-{ndim} array")
    # Immutable bytes own the buffer: callers cannot re-enable writeability and
    # mutate evidence behind its accepted fingerprint.
    return np.frombuffer(result.tobytes(), dtype=result.dtype).reshape(result.shape)


def _symmetric(value, name, strict):
    matrix = _array(value, 2, name)
    if matrix.shape[0] != matrix.shape[1] or not np.allclose(matrix, matrix.T, rtol=1e-12, atol=1e-14):
        raise ValueError(f"{name}: expected symmetric square matrix")
    if strict:
        np.linalg.cholesky(matrix)  # No flooring or invented precision.
    elif matrix.size and np.linalg.eigvalsh(matrix).min() < 0.0:
        raise ValueError(f"{name}: negative shared variance")
    return matrix


def gaussian_nll(residual, covariance):
    """Normalized Gaussian NLL with state-dependent covariance supplied by caller."""
    residual = _array(residual, 1, "residual")
    covariance = _symmetric(covariance, "covariance", True)
    if covariance.shape != (len(residual), len(residual)):
        raise ValueError("covariance dimension mismatch")
    lower = np.linalg.cholesky(covariance)
    whitened = np.linalg.solve(lower, residual)
    return float(.5 * (whitened @ whitened + 2 * np.log(np.diag(lower)).sum()
                       + len(residual) * np.log(2 * np.pi)))


def gaussian_nll_derivative(residual, covariance, residual_derivative, covariance_derivative):
    """One-coordinate derivative, including the logdet derivative of V(x)."""
    r = _array(residual, 1, "residual")
    v = _symmetric(covariance, "covariance", True)
    dr = _array(residual_derivative, 1, "residual_derivative")
    dv = _array(covariance_derivative, 2, "covariance_derivative")
    if dr.shape != r.shape or v.shape != (len(r), len(r)) or dv.shape != v.shape:
        raise ValueError("derivative dimension mismatch")
    if not np.allclose(dv, dv.T, rtol=1e-12, atol=1e-14):
        raise ValueError("covariance derivative must be symmetric")
    alpha = np.linalg.solve(v, r)
    return float(dr @ alpha - .5 * alpha @ dv @ alpha + .5 * np.trace(np.linalg.solve(v, dv)))


@dataclass(frozen=True)
class SharedVariable:
    variable_id: str
    physical_meaning: str
    source_identity: str
    lifetime: str
    observability: str = "unknown"
    calibration_status: str = "unvalidated"
    independent_calibration_sha256: str = ""

    def __post_init__(self):
        if not all((self.variable_id, self.physical_meaning, self.source_identity, self.lifetime)):
            raise ValueError("shared variable needs physical source and lifetime")
        if self.calibration_status not in ("unknown", "unvalidated", "independent_calibration_supplied"):
            raise ValueError("unsupported calibration status")
        if self.calibration_status == "independent_calibration_supplied" and (
                len(self.independent_calibration_sha256) != 64 or
                any(c not in "0123456789abcdef" for c in self.independent_calibration_sha256)):
            raise ValueError("independent calibration requires an explicit SHA256")


@dataclass(frozen=True)
class EvidenceBlock:
    evidence_id: str
    measurement_ids: Tuple[str, ...]  # Physical sample IDs; support reuse is separate.
    query_source: str
    geometry_identity: str
    reference_time_ns: int
    y: object
    J: object
    B: object
    R: object  # Conditional independent remainder only; excludes eta uncertainty.

    def __post_init__(self):
        object.__setattr__(self, 'measurement_ids', tuple(self.measurement_ids))
        for name, rank in (("y", 1), ("J", 2), ("B", 2)):
            object.__setattr__(self, name, _array(getattr(self, name), rank, name))
        object.__setattr__(self, "R", _symmetric(self.R, "R", True))
        n = len(self.y)
        if not n or len(self.measurement_ids) != n or len(set(self.measurement_ids)) != n:
            raise ValueError("physical sample IDs must be nonempty and unique")
        if any(not isinstance(k, str) or not k for k in self.measurement_ids):
            raise ValueError("physical sample IDs must be nonempty strings")
        if self.J.shape[0] != n or self.B.shape[0] != n or self.R.shape != (n, n):
            raise ValueError("evidence dimensions disagree")
        if not all((self.evidence_id, self.query_source, self.geometry_identity)) or self.reference_time_ns < 0:
            raise ValueError("missing evidence/source/geometry/time identity")

    def fingerprint(self):
        h = hashlib.sha256(json.dumps([self.evidence_id, self.measurement_ids, self.query_source,
                                     self.geometry_identity, self.reference_time_ns]).encode())
        for matrix in (self.y, self.J, self.B, self.R):
            h.update(str(matrix.shape).encode()); h.update(matrix.tobytes())
        return h.hexdigest()


class PersistentSharedGaussian:
    """Exact dense shadow likelihood over a bounded, immutable geometry version.

    x is one caller-defined fixed vector across the whole window (it may contain
    multiple pose increments). Re-linearizing J/B or rebuilding geometry requires
    a NEW model/identity; this class does not carry an invalid stale prior.
    """
    shadow_only = True

    def __init__(self, state_dimension, variables, shared_covariance, geometry_identity,
                 linearization_identity, max_observations=2048):
        self.variables = tuple(variables)
        if len({v.variable_id for v in self.variables}) != len(self.variables):
            raise ValueError("duplicate shared variable")
        self.shared_covariance = _symmetric(shared_covariance, "shared_covariance", False)
        if self.shared_covariance.shape != (len(self.variables), len(self.variables)):
            raise ValueError("shared variable dimension mismatch")
        if state_dimension <= 0 or max_observations <= 0 or not geometry_identity or not linearization_identity:
            raise ValueError("missing state/budget/geometry/linearization identity")
        self.state_dimension = state_dimension
        self.geometry_identity = geometry_identity
        self.linearization_identity = linearization_identity
        self.max_observations = max_observations
        self._blocks = []
        self._fingerprints = {}
        self._measurement_ids = set()

    def append(self, block):
        """Atomic accept, exact retransmission no-op, conflicting/duplicate evidence reject."""
        fingerprint = block.fingerprint()
        if block.evidence_id in self._fingerprints:
            if self._fingerprints[block.evidence_id] != fingerprint:
                raise ValueError("same evidence ID with changed content")
            return False
        if block.geometry_identity != self.geometry_identity:
            raise ValueError("geometry changed: rebuild the joint model explicitly")
        if block.J.shape[1] != self.state_dimension or block.B.shape[1] != len(self.variables):
            raise ValueError("state/shared dimension mismatch")
        if self._measurement_ids.intersection(block.measurement_ids):
            raise ValueError("physical evidence already consumed under another evidence ID")
        if self.observation_count + len(block.y) > self.max_observations:
            raise ValueError("shadow observation budget exceeded; history was retained")
        self._blocks.append(block)
        self._fingerprints[block.evidence_id] = fingerprint
        self._measurement_ids.update(block.measurement_ids)
        return True

    @property
    def observation_count(self):
        return len(self._measurement_ids)

    def _system(self, blocks):
        y = np.concatenate([b.y for b in blocks])
        j = np.concatenate([b.J for b in blocks])
        bmat = np.concatenate([b.B for b in blocks])
        r = np.zeros((len(y), len(y))); start = 0
        for block in blocks:
            end = start + len(block.y); r[start:end, start:end] = block.R; start = end
        return y, j, bmat, r

    def nll(self, state, prefix=None):
        state = _array(state, 1, "state")
        if state.shape != (self.state_dimension,):
            raise ValueError("state dimension mismatch")
        if prefix is not None and not 0 <= prefix <= len(self._blocks):
            raise ValueError("invalid block prefix")
        blocks = self._blocks if prefix is None else self._blocks[:prefix]
        if not blocks:
            return 0.
        y, j, bmat, r = self._system(blocks)
        return gaussian_nll(y - j @ state, r + bmat @ self.shared_covariance @ bmat.T)

    def last_conditional_nll(self, state):
        if not self._blocks:
            return 0.
        return self.nll(state) - self.nll(state, len(self._blocks) - 1)

    def audit_record(self):
        return {"shadow_only": True, "geometry_identity": self.geometry_identity,
                "linearization_identity": self.linearization_identity,
                "observations": self.observation_count, "blocks": len(self._blocks),
                "evidence_sha256": dict(self._fingerprints),
                "shared_variables": [vars(v) for v in self.variables],
                "physical_accuracy_calibration": "unknown",
                "model": "joint Gaussian; persistent eta; fixed declared linearization"}
