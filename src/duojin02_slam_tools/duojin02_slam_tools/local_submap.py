"""Opt-in rigid local-map shadow model; no production global optimizer change.

Internal endpoint geometry is immutable. Global pose revisions are separate from
geometry revisions. Source overlap is recorded, never converted into invented
independence or a correlation coefficient.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from collections import deque
from functools import cached_property
import hashlib
import json
import math
import time
from typing import Callable


def _digest(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return tuple((c*x-s*y+pose[0], s*x+c*y+pose[1]) for x, y in points)


def relative(world, anchor):
    c, s = math.cos(anchor[2]), math.sin(anchor[2])
    x, y = world[0]-anchor[0], world[1]-anchor[1]
    return (c*x+s*y, -s*x+c*y, math.atan2(math.sin(world[2]-anchor[2]), math.cos(world[2]-anchor[2])))


@dataclass(frozen=True)
class RawObservation:
    source_id: str
    raw_stamp_ns: int
    physical_reference_ns: int
    geometry_kind: str
    points: tuple[tuple[float, float], ...]
    beam_indices: tuple[int, ...]
    calibration_version: str = "unidentified"
    ray_origins: tuple[tuple[float, float], ...] | None = None
    source_geometry_hash: str = "unknown"

    def __post_init__(self):
        object.__setattr__(self, "points", tuple(tuple(p) for p in self.points))
        object.__setattr__(self, "beam_indices", tuple(self.beam_indices))
        if self.ray_origins is not None:
            object.__setattr__(self, "ray_origins", tuple(tuple(p) for p in self.ray_origins))
            if len(self.ray_origins) != len(self.points) or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in self.ray_origins):
                raise ValueError("ray origins must preserve the endpoint beam roster")
        if not self.source_id or not self.geometry_kind or len(self.points) != len(self.beam_indices):
            raise ValueError("source, geometry and complete endpoint/beam roster required")
        if not self.points or len(set(self.beam_indices)) != len(self.beam_indices):
            raise ValueError("empty or repeated source beam roster")
        if any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in self.points):
            raise ValueError("raw endpoint geometry must be finite")

    @classmethod
    def from_dict(cls, value):
        return cls(value["source_id"], value["raw_stamp_ns"], value["physical_reference_ns"],
                   value["geometry_kind"], tuple(tuple(p) for p in value["points"]), tuple(value["beam_indices"]),
                   value.get("calibration_version", "unidentified"), value.get("ray_origins"),
                   _digest({"measurement_payload_hex": value["measurement_payload_hex"], "sensor_geometry_config": value["sensor_geometry_config"]})
                   if "measurement_payload_hex" in value else value.get("source_geometry_hash", "unknown"))

    @cached_property
    def identity(self):
        return _digest({"source_id": self.source_id, "raw_stamp_ns": self.raw_stamp_ns,
                        "physical_reference_ns": self.physical_reference_ns, "geometry_kind": self.geometry_kind,
                        "points": self.points, "beam_indices": self.beam_indices,
                        "ray_origins": self.ray_origins, "source_geometry_hash": self.source_geometry_hash,
                        "calibration_version": self.calibration_version})


@dataclass(frozen=True)
class FrozenScan:
    observation: RawObservation
    local_pose: tuple[float, float, float]

    def __post_init__(self):
        object.__setattr__(self, "local_pose", tuple(self.local_pose))
        if len(self.local_pose) != 3 or not all(math.isfinite(v) for v in self.local_pose):
            raise ValueError("finite immutable internal scan pose required")


@dataclass(frozen=True)
class QueryReceipt:
    query_source_id: str
    query_identity: str
    reference_geometry_version: str
    reference_source_ids: tuple[str, ...]


@dataclass(frozen=True)
class RigidSubmap:
    submap_id: str
    scans: tuple[FrozenScan, ...]
    world_pose: tuple[float, float, float]
    pose_version: int = 0

    def __post_init__(self):
        object.__setattr__(self, "scans", tuple(self.scans))
        object.__setattr__(self, "world_pose", tuple(self.world_pose))
        sources = self.source_ids
        if len(set(sources)) != len(sources):
            raise ValueError("one raw source cannot be inserted twice in one geometry")
        if not self.submap_id or len(self.world_pose) != 3 or not all(math.isfinite(v) for v in self.world_pose):
            raise ValueError("finite rigid pose and submap identity required")

    @property
    def source_ids(self):
        return tuple(scan.observation.source_id for scan in self.scans)

    @cached_property
    def geometry_version(self):
        return _digest([(s.observation.identity, s.local_pose) for s in self.scans])

    def moved(self, world_pose):
        """Only a common rigid transform changes; internal scan poses stay fixed."""
        return replace(self, world_pose=tuple(world_pose), pose_version=self.pose_version+1)

    def query_receipt(self, observation):
        if observation.source_id in self.source_ids:
            raise ValueError("query source already belongs to reference geometry")
        return QueryReceipt(observation.source_id, observation.identity, self.geometry_version, self.source_ids)

    def overlap(self, other):
        return tuple(sorted(set(self.source_ids).intersection(other.source_ids)))


class RigidSubmapBuilder:
    """Finite geometry budget freezes a map; callers retain later accepted work."""
    def __init__(self, submap_id, world_pose, max_scans=40, max_points=40000):
        if max_scans < 1 or max_points < 2:
            raise ValueError("positive geometry budget required")
        self.submap_id, self.world_pose = submap_id, tuple(world_pose)
        self.max_scans, self.max_points = max_scans, max_points
        self._scans = []
        self._identities = {}
        self.insertion_receipts = {}
        self._frozen = False

    def view(self):
        return RigidSubmap(self.submap_id, tuple(self._scans), self.world_pose)

    def insert_once(self, observation, world_scan_pose, receipt):
        if self._frozen:
            raise ValueError("frozen geometry cannot be mutated; open a new submap")
        if receipt.query_identity != observation.identity or receipt.query_source_id != observation.source_id:
            raise ValueError("query receipt does not identify this exact raw scan")
        if observation.source_id in receipt.reference_source_ids:
            raise ValueError("self-matching query cannot enter geometry")
        if observation.source_id in self._identities:
            if self._identities[observation.source_id] != observation.identity:
                raise ValueError("raw source identity reused with changed measurement")
            return False
        view = self.view()
        if receipt.reference_geometry_version != view.geometry_version or receipt.reference_source_ids != view.source_ids:
            raise ValueError("stale or external query receipt does not match current builder geometry")
        # Capacity is checked before mutation; callers keep this accepted update.
        if len(self._scans) >= self.max_scans or sum(len(s.observation.points) for s in self._scans)+len(observation.points) > self.max_points:
            raise OverflowError("geometry budget reached; freeze and retain update for next map")
        self._scans.append(FrozenScan(observation, relative(world_scan_pose, self.world_pose)))
        self._identities[observation.source_id] = observation.identity
        self.insertion_receipts[observation.source_id] = receipt
        return True

    def freeze(self):
        self._frozen = True
        return self.view()


@dataclass(frozen=True)
class AcceptedUpdate:
    update_id: str
    payload: object


class BudgetedUpdateQueue:
    """Retain every accepted update until an entire callback succeeds.

    This is a shadow scheduling primitive, not a global solver or a fixed-lag
    marginalizer. One callback is indivisible and may exceed the time budget.
    """
    def __init__(self):
        self._pending = deque()
        self._accepted = {}
        self.completed_ids = []

    def accept(self, update):
        if update.update_id in self._accepted:
            if self._accepted[update.update_id] != update:
                raise ValueError("accepted update ID reused with different content")
            return False
        self._accepted[update.update_id] = update
        self._pending.append(update)
        return True

    @property
    def pending_ids(self):
        return tuple(x.update_id for x in self._pending)

    def run_budget(self, callback: Callable, max_updates=1, budget_seconds=0.01):
        if max_updates < 1 or budget_seconds <= 0:
            raise ValueError("positive scheduling budget required")
        start, count = time.monotonic(), 0
        while self._pending and count < max_updates and time.monotonic()-start < budget_seconds:
            update = self._pending[0]
            # On exception it remains pending. callback must commit atomically or
            # be idempotent; this queue cannot undo arbitrary external effects.
            callback(update)
            self._pending.popleft()
            self.completed_ids.append(update.update_id)
            count += 1
        return {"completed": count, "pending": len(self._pending), "elapsed_seconds": time.monotonic()-start,
                "accepted": len(self._accepted), "all_accepted_accounted": len(self._accepted) == len(self.completed_ids)+len(self._pending)}


@dataclass(frozen=True)
class ScanSubmapObservation:
    """A shadow local measurement tied to the exact rigid reference geometry.

    This does not register a Ceres factor and provides no calibrated covariance.
    Pose-only global revisions preserve support; rebuilding internal geometry
    invalidates the measurement and requires recomputation.
    """
    query_source_id: str
    query_identity: str
    submap_id: str
    geometry_version: str
    reference_source_ids: tuple[str, ...]
    query_local_pose: tuple[float, float, float]
    conditional_shape: tuple[tuple[float, float, float], ...]
    covariance_calibrated: bool = False

    @classmethod
    def from_shadow(cls, observation, submap, result):
        if observation.source_id in submap.source_ids:
            raise ValueError("query cannot already belong to reference geometry")
        if result['geometry_version'] != submap.geometry_version:
            raise ValueError("estimate geometry does not match its immutable support")
        return cls(observation.source_id, observation.identity, submap.submap_id,
                   submap.geometry_version, submap.source_ids, tuple(result['local_pose']),
                   tuple(tuple(row) for row in result['conditional_shape']))

    def validate_support(self, submap):
        if submap.submap_id != self.submap_id or submap.geometry_version != self.geometry_version or submap.source_ids != self.reference_source_ids:
            raise ValueError("scan-to-submap observation invalidated by internal geometry revision")
        return True
