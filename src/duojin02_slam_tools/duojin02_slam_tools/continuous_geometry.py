"""Continuous point-to-segment local matching on actual scan endpoints.

Research/shadow path only. Association is recomputed locally, rank is reported
without diagonal flooring, and the Hessian is conditional shape, not calibrated
physical covariance. It never updates PF weights or the Karto graph.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import time
import numpy as np
from .local_submap import transform, relative


@dataclass(frozen=True)
class Surface:
    surface_id: str
    source_id: str
    beam_indices: tuple[int, int]
    a: tuple[float, float]
    b: tuple[float, float]


def surfaces_from_submap(submap, max_segment_length=0.25, min_segment_length=0.005):
    surfaces = []
    for scan in submap.scans:
        points = transform(scan.observation.points, scan.local_pose)
        indices = scan.observation.beam_indices
        for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            if indices[i+1] != indices[i]+1 or not min_segment_length <= length <= max_segment_length:
                continue
            surfaces.append(Surface(f"{scan.observation.identity}:{indices[i]}:{indices[i+1]}",
                                    scan.observation.source_id, (indices[i], indices[i+1]), a, b))
    return tuple(surfaces)


class SurfaceIndex:
    """Spatial candidate index; continuous segment residuals stay unquantized."""
    def __init__(self, surfaces, cell_size=0.25):
        if cell_size <= 0:
            raise ValueError("positive index cell size required")
        self.surfaces, self.cell_size, self.cells = surfaces, cell_size, {}
        for i, s in enumerate(surfaces):
            lo = np.floor(np.minimum(s.a, s.b)/cell_size).astype(int)
            hi = np.floor(np.maximum(s.a, s.b)/cell_size).astype(int)
            for x in range(lo[0], hi[0]+1):
                for y in range(lo[1], hi[1]+1):
                    self.cells.setdefault((x, y), []).append(i)

    def correspondences(self, transformed, max_distance):
        chosen = []
        radius = int(math.ceil(max_distance/self.cell_size))
        for beam_row, point in enumerate(transformed):
            cell = np.floor(point/self.cell_size).astype(int)
            ids = set()
            for x in range(cell[0]-radius, cell[0]+radius+1):
                for y in range(cell[1]-radius, cell[1]+radius+1):
                    ids.update(self.cells.get((x, y), ()))
            best = None
            for i in sorted(ids):
                s = self.surfaces[i]
                a, b = np.asarray(s.a), np.asarray(s.b)
                d = b-a
                u = float((point-a)@d/(d@d))
                if not 0.0 <= u <= 1.0:
                    continue  # endpoint has no point-to-line validity here
                n = np.array([-d[1], d[0]])/np.linalg.norm(d)
                residual = float(n@(point-a))
                distance = abs(residual)
                if distance <= max_distance and (best is None or distance < best[0]):
                    best = (distance, i, n, residual)
            if best is not None:
                chosen.append((beam_row, best[1], best[2], best[3]))
        return chosen


def residual_jacobian(point, local_pose, surface):
    """Fixed-association residual and derivative in local x/y/yaw coordinates."""
    c, s = math.cos(local_pose[2]), math.sin(local_pose[2])
    p = np.asarray(point)
    rotated = np.array([c*p[0]-s*p[1], s*p[0]+c*p[1]])
    a, b = np.asarray(surface.a), np.asarray(surface.b)
    d = b-a
    n = np.array([-d[1], d[0]])/np.linalg.norm(d)
    residual = float(n@(rotated+np.asarray(local_pose[:2])-a))
    jacobian = np.array([n[0], n[1], n@np.array([-rotated[1], rotated[0]])])
    return residual, jacobian


def refine_shadow(observation, submap, initial_world_pose, *, query_stride=4,
                  max_iterations=8, max_distance=0.12, huber_scale=0.03,
                  task_length_scale=1.0, min_eigenvalue=1e-6, min_support_retention=0.9):
    if query_stride < 1 or max_iterations < 1 or min(max_distance, huber_scale, task_length_scale, min_eigenvalue) <= 0 or not 0 < min_support_retention <= 1:
        raise ValueError("positive matching and dimensionless task scales required")
    receipt = submap.query_receipt(observation)  # fail before self-match can score
    started = time.monotonic()
    surfaces = surfaces_from_submap(submap)
    index = SurfaceIndex(surfaces)
    pose = np.asarray(relative(initial_world_pose, submap.world_pose), dtype=float)
    points = np.asarray(observation.points)[::query_stride]
    beam_ids = observation.beam_indices[::query_stride]
    iterations, status = 0, "insufficient_support"
    scale = np.diag([task_length_scale, task_length_scale, 1.0])

    def evaluate(p):
        transformed = np.asarray(transform(points, p))
        pairs = index.correspondences(transformed, max_distance)
        rows, residuals = [], []
        for row, sid, _, _ in pairs:
            residual, jacobian = residual_jacobian(points[row], p, surfaces[sid])
            rows.append(jacobian); residuals.append(residual)
        return pairs, np.asarray(residuals), np.asarray(rows).reshape((-1, 3))

    def robust_cost(residuals):
        a = np.abs(residuals)
        return float(np.sum(np.where(a <= huber_scale, 0.5*a*a, huber_scale*(a-0.5*huber_scale))))

    initial_pairs, _, _ = evaluate(pose)
    initial_support = {row for row, _, _, _ in initial_pairs}
    for iteration in range(max_iterations):
        pairs, residuals, jacobian = evaluate(pose)
        if len(pairs) < 6:
            break
        weights = np.minimum(1.0, huber_scale/np.maximum(np.abs(residuals), 1e-30))
        hessian = jacobian.T@(weights[:, None]*jacobian)
        eigenvalues = np.linalg.eigvalsh(scale@hessian@scale)
        if eigenvalues[0] <= min_eigenvalue:
            status = "weak_geometry"
            break
        step = np.linalg.solve(hessian, -jacobian.T@(weights*residuals))
        # Bounded single-mode refinement, with fixed-association line search.
        norm = max(np.linalg.norm(step[:2])/0.03, abs(step[2])/0.03, 1.0)
        step /= norm
        old_cost = robust_cost(residuals)
        accepted = False
        for fraction in (1.0, 0.5, 0.25, 0.125):
            candidate = pose+fraction*step
            fixed_residuals = np.array([residual_jacobian(points[row], candidate, surfaces[sid])[0] for row,sid,_,_ in pairs])
            if robust_cost(fixed_residuals) <= old_cost+1e-15:
                pose = candidate; accepted = True; break
        iterations = iteration+1
        status = "local_mode_only" if accepted else "line_search_failed"
        if not accepted or np.linalg.norm(fraction*step) < 1e-7:
            break
    pairs, residuals, jacobian = evaluate(pose)
    hessian = jacobian.T@jacobian
    eigenvalues = np.linalg.eigvalsh(scale@hessian@scale)
    observable = len(pairs) >= 6 and eigenvalues[0] > min_eigenvalue
    if not observable:
        status = "weak_geometry" if len(pairs) >= 6 else "insufficient_support"
    final_support = {row for row, _, _, _ in pairs}
    retention = len(initial_support & final_support)/len(initial_support) if initial_support else 0.0
    if initial_support and retention < min_support_retention:
        status = "support_lost"
    detail = []
    for k, (row, sid, _, _) in enumerate(pairs):
        surface = surfaces[sid]
        detail.append({"query_source_id": observation.source_id, "query_beam_index": beam_ids[row],
                       "surface_id": surface.surface_id, "surface_source_id": surface.source_id,
                       "surface_beam_indices": surface.beam_indices, "residual_m": float(residuals[k]),
                       "jacobian": jacobian[k].tolist(), "geometry_version": submap.geometry_version,
                       "pose_version": submap.pose_version, "conditional_only": True,
                       "calibration_version": observation.calibration_version,
                       "source_geometry_hash": observation.source_geometry_hash,
                       "ray_origin_available": observation.ray_origins is not None,
                       "visibility": "unvalidated"})
    return {"local_pose": pose.tolist(), "status": status, "observable": bool(observable),
            "geometry_version": submap.geometry_version, "pose_version": submap.pose_version,
            "reference_sources": submap.source_ids, "query_receipt": receipt,
            "conditional_shape": hessian.tolist(), "dimensionless_eigenvalues": eigenvalues.tolist(),
            "task_length_scale_m": task_length_scale, "conditional_only": True, "covariance_calibrated": False,
            "visibility": "unvalidated", "usable_for_precision": False,
            "support_retention": retention, "minimum_support_retention": min_support_retention,
            "initial_matched_points": len(initial_support), "support_changed": initial_support != final_support,
            "iterations": iterations, "surface_count": len(surfaces), "query_points": len(points),
            "matched_points": len(pairs), "rms_residual_m": float(np.sqrt(np.mean(residuals**2))) if len(pairs) else None,
            "elapsed_seconds": time.monotonic()-started, "residuals": detail}
