"""ROS-independent scan/map quality evidence for advisory diagnostics.

AMCL/Karto use nearest grid indices and store the first cell centre in origin.
No map or pose is shifted here. `occupancy_cell` is an explicit alternative for
backends which interpret origin as the lower cell corner. Unknown cells and
unobserved (+Inf) beams never provide positive localization evidence.
"""
from bisect import bisect_left
from dataclasses import dataclass
import math
from pathlib import Path
from types import MappingProxyType

import yaml


def _parameter_defaults():
    # Source checkouts keep the same canonical YAML beside the Python package;
    # installed packages use its ament share location.
    config = Path(__file__).resolve().parents[1] / 'config/localization_monitor.yaml'
    if not config.is_file():
        from ament_index_python.packages import get_package_share_directory
        config = Path(get_package_share_directory('duojin02_bringup')) / 'config/localization_monitor.yaml'
    return yaml.safe_load(config.read_text())['localization_monitor']['ros__parameters']


PARAMETER_DEFAULTS = MappingProxyType(_parameter_defaults())


@dataclass(frozen=True)
class ScorePolicy:
    min_hits: int = PARAMETER_DEFAULTS['min_hits']
    hit_fraction: float = PARAMETER_DEFAULTS['hit_fraction']
    known_fraction: float = PARAMETER_DEFAULTS['known_fraction']
    max_contradiction_fraction: float = PARAMETER_DEFAULTS['max_contradiction_fraction']
    endpoint_tolerance: float = PARAMETER_DEFAULTS['endpoint_tolerance']
    ray_margin: float = PARAMETER_DEFAULTS['ray_margin']
    max_ray_cells: int = PARAMETER_DEFAULTS['max_ray_cells']
    max_range: float = PARAMETER_DEFAULTS['max_range']
    max_beams: int = PARAMETER_DEFAULTS['max_beams']


class MapEvidence:
    def __init__(self, width, height, resolution, origin, data,
                 convention='amcl_nearest'):
        if convention not in ('amcl_nearest', 'occupancy_cell'):
            raise ValueError('Unsupported map cell convention')
        if width <= 0 or height <= 0 or resolution <= 0 or len(data) != width * height:
            raise ValueError('Invalid occupancy map geometry')
        if not all(math.isfinite(x) for x in (*origin, resolution)):
            raise ValueError('Non-finite map geometry')
        if convention == 'amcl_nearest' and abs(origin[2]) > 1e-6:
            raise ValueError('Pinned AMCL ignores rotated map origins; fixed-map yaw must be zero')
        self.width, self.height, self.resolution = width, height, resolution
        # Pinned AMCL's convertMap treats only 0 and 100 as known; scaled or
        # raw intermediate probabilities must not inflate our known support.
        self.origin, self.convention = origin, convention
        self.data = [v if v in (0, 100) else -1 for v in data]
        self.c, self.s = math.cos(origin[2]), math.sin(origin[2])
        self.centre_offset = 0.0 if convention == 'amcl_nearest' else 0.5
        self.rows = [[] for _ in range(height)]
        for i, value in enumerate(self.data):
            if value >= 65:
                self.rows[i // width].append(i % width)

    def grid_point(self, x, y):
        dx, dy = x - self.origin[0], y - self.origin[1]
        return ((self.c * dx + self.s * dy) / self.resolution,
                (-self.s * dx + self.c * dy) / self.resolution)

    def cell(self, gx, gy):
        offset = 0.5 if self.convention == 'amcl_nearest' else 0.0
        ix, iy = math.floor(gx + offset), math.floor(gy + offset)
        if not 0 <= ix < self.width or not 0 <= iy < self.height:
            return -1
        return self.data[iy * self.width + ix]

    def near_occupied(self, gx, gy, tolerance):
        # Distance to the occupied cell area, rather than just its centre.
        radius = tolerance / self.resolution
        cx, cy = gx - self.centre_offset, gy - self.centre_offset
        low = max(0, math.floor(cy - radius - 0.5))
        high = min(self.height - 1, math.ceil(cy + radius + 0.5))
        for iy in range(low, high + 1):
            dy = max(0.0, abs(cy - iy) - 0.5)
            if dy > radius:
                continue
            reach = math.sqrt(max(0.0, radius * radius - dy * dy)) + 0.5
            row = self.rows[iy]
            j = bisect_left(row, cx - reach)
            if j < len(row) and row[j] <= cx + reach:
                return True
        return False

    def ray_cells(self, x, y, dx, dy, start, end, interior=False):
        """Traverse every grid cell intersected by a free-ray segment (DDA)."""
        gx, gy = self.grid_point(x, y)
        vx = (self.c * dx + self.s * dy) / self.resolution
        vy = (-self.s * dx + self.c * dy) / self.resolution
        offset = 0.5 if self.convention == 'amcl_nearest' else 0.0
        sx, sy = gx + start * vx + offset, gy + start * vy + offset
        ix, iy = math.floor(sx), math.floor(sy)
        stepx, stepy = (1 if vx > 0 else -1), (1 if vy > 0 else -1)
        tx = start + ((ix + 1 - sx) / vx if vx > 0 else (ix - sx) / vx) if vx else math.inf
        ty = start + ((iy + 1 - sy) / vy if vy > 0 else (iy - sy) / vy) if vy else math.inf
        dtx, dty = abs(1 / vx) if vx else math.inf, abs(1 / vy) if vy else math.inf
        t = start
        while t < end:
            interval_end = min(tx, ty, end)
            if interval_end > t:
                value = self.data[iy * self.width + ix] if 0 <= ix < self.width and 0 <= iy < self.height else -1
                if interior:
                    # Near-ray occupied evidence requires positive cell
                    # interior, rather than an edge or zero-length contact.
                    mid = (t + interval_end) / 2
                    mx, my = gx + mid * vx + offset, gy + mid * vy + offset
                    if not (ix < mx < ix + 1 and iy < my < iy + 1):
                        value = -1
                yield value
            else:
                # An exact reverse-boundary start has zero length. Extremely
                # small cells can also stop floating-point time advancement.
                # Count either transition as unknown so the caller's budget
                # always bounds DDA work; neither adds occupied evidence.
                yield -1
            if tx < ty:
                t, ix, tx = tx, ix + stepx, tx + dtx
            elif ty < tx:
                t, iy, ty = ty, iy + stepy, ty + dty
            else:
                # Crossing a corner touches neither adjacent cell with positive
                # ray length; visit the diagonally entered cell.
                t, ix, iy, tx, ty = tx, ix + stepx, iy + stepy, tx + dtx, ty + dty

    def score(self, ranges, angle_min, angle_increment, range_min, range_max,
              laser_pose, policy=ScorePolicy()):
        if (not ranges or not all(math.isfinite(x) for x in (*laser_pose, angle_min, angle_increment, range_min, range_max)) or
                not 0 <= range_min < range_max or angle_increment == 0 or
                not math.pi <= abs(angle_increment) * (len(ranges) - 1) <= 2 * math.pi + 0.05):
            return {'valid': False, 'reason': 'invalid_scan_geometry', 'hits': 0}
        # Malformed rays outside the scoring subset still invalidate the scan.
        if any(r is None or math.isnan(r) or r == -math.inf or r < range_min for r in ranges):
            return {'valid': False, 'reason': 'invalid_scan_ray', 'hits': 0}
        # Select uniformly before filtering: invalid/unknown beams cannot cause
        # a favourable subset to replace the requested angular evidence.
        step = max(1, math.ceil(len(ranges) / policy.max_beams))
        hits = known = matched = contradictions = ray_known = ray_total = finite_ray_known = finite_ray_total = invalid = unobserved = 0
        compared = policy_clipped = ray_work = 0
        lx, ly, yaw = laser_pose
        if self.cell(*self.grid_point(lx, ly)) >= 65:
            return dict(valid=False, reason='laser_origin_occupied', hits=0)
        limit = min(range_max, policy.max_range)
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if r is None or math.isnan(r) or r == -math.inf or r < range_min:
                invalid += 1
                continue
            # The N10 publisher uses Inf for every empty bin after range
            # filtering, so it cannot certify free space or a distant no echo.
            # Out-of-sensor-range values likewise carry no geometric evidence.
            if r == math.inf or r > range_max:
                unobserved += 1
                continue
            compared += 1
            no_hit = r >= limit
            if no_hit:
                policy_clipped += 1
                r = limit
            else:
                hits += 1
            a = yaw + angle_min + i * angle_increment
            dx, dy = math.cos(a), math.sin(a)
            # Valid finite returns certify their preceding ray. Preserve near
            # contradictions and the endpoint margin even when policy-clipped.
            blind_end = min(range_min, max(0.0, r - policy.ray_margin))
            blind_blocked = False
            for value in self.ray_cells(lx, ly, dx, dy, 0.0, blind_end, interior=True):
                ray_work += 1
                if ray_work > policy.max_ray_cells:
                    return dict(valid=False, reason='map_ray_work_budget_exceeded', hits=hits)
                if value >= 65:
                    blind_blocked = True
                    break
            gx, gy = self.grid_point(lx + r * dx, ly + r * dy)
            if not no_hit and self.cell(gx, gy) >= 0:
                known += 1
                matched += int(self.near_occupied(gx, gy, policy.endpoint_tolerance))
            contradictory = blind_blocked
            for value in self.ray_cells(lx, ly, dx, dy, range_min, r - policy.ray_margin):
                ray_total += 1
                ray_work += 1
                if ray_work > policy.max_ray_cells:
                    return dict(valid=False, reason='map_ray_work_budget_exceeded', hits=hits)
                ray_known += int(value >= 0)
                if not no_hit:
                    finite_ray_total += 1
                    finite_ray_known += int(value >= 0)
                contradictory |= value >= 65
            contradictions += int(contradictory)
        support = matched / hits if hits else 0.0
        known_fraction = min(known / hits if hits else 0.0,
                             finite_ray_known / finite_ray_total if finite_ray_total else 0.0)
        contradiction_fraction = contradictions / compared if compared else 0.0
        good = (invalid == 0 and hits >= policy.min_hits and
                support >= policy.hit_fraction and known_fraction >= policy.known_fraction and
                contradiction_fraction <= policy.max_contradiction_fraction)
        reason = ('consistent' if good else 'insufficient_observed_known_support'
                  if hits < policy.min_hits or known_fraction < policy.known_fraction
                  else 'scan_map_inconsistent_or_occluded')
        return dict(valid=good, reason=reason,
                    hits=hits, matched=matched, known=known, invalid=invalid, unobserved=unobserved,
                    policy_clipped=policy_clipped, compared=compared, contradictions=contradictions, ray_work=ray_work,
                    hit_fraction=support, known_fraction=known_fraction,
                    all_ray_known_fraction=ray_known/ray_total if ray_total else 0.0,
                    contradiction_fraction=contradiction_fraction)

