import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/duojin02_bringup"))
from duojin02_bringup.localization_quality import ScorePolicy, MapEvidence


def rings(inner=False, unknown=False):
    size, resolution = 500, 0.01
    data = []
    for y in range(size):
        for x in range(size):
            r = math.hypot(-2.5 + x * resolution, -2.5 + y * resolution)
            data.append(
                -1
                if unknown
                else 100 if abs(r - 2.0) < 0.015 or inner and abs(r - 0.37) < 0.005 else 0
            )
    return MapEvidence(size, size, resolution, (-2.5, -2.5, 0), data)


def score(grid, ranges, pose=(0, 0, 0), increment=2 * math.pi / 90, minimum=0.3):
    return grid.score(
        ranges, -math.pi, increment, minimum, 8.0, pose, ScorePolicy(max_ray_cells=100000)
    )


def test_correct_match_and_wrong_low_uncertainty_pose():
    grid = rings()
    assert score(grid, [2.0] * 90)["valid"]
    assert not score(grid, [2.0] * 90, (1.0, 0, 0))["valid"]


def test_unknown_no_return_and_observed_thin_wall_have_distinct_evidence():
    grid = rings()
    v = score(grid, [2.0] * 45 + [math.inf] * 45)

    assert v["valid"] and v["compared"] == 45 and v["unobserved"] == 45
    assert v["contradictions"] == 0 and v["contradiction_fraction"] == 0.0
    v = score(rings(inner=True), [2.0] * 90)
    assert not v["valid"] and v["contradiction_fraction"] > 0.9


def test_unobserved_unknown_and_invalid_unsampled_ray_cannot_support_pose():
    assert not score(rings(), [math.inf] * 90)["valid"]
    assert not score(rings(unknown=True), [2.0] * 90)["valid"]
    grid = rings()
    scaled = [100 if v == 100 else 50 for v in grid.data]
    assert not score(MapEvidence(500, 500, 0.01, (-2.5, -2.5, 0), scaled), [2.0] * 90)["valid"]
    values = [2.0] * 540
    values[1] = -math.inf
    assert not score(rings(), values, increment=2 * math.pi / 540)["valid"]
    values[1] = math.nan
    assert not score(rings(), values, increment=2 * math.pi / 540)["valid"]
    assert not score(rings(), [2.0] * 90, increment=0)["valid"]
    assert not score(rings(), [2.0] * 90, minimum=-0.1)["valid"]


def test_map_convention_and_rotated_amcl_are_explicit():
    near = MapEvidence(3, 3, 0.02, (0, 0, 0), [0] * 9)
    normal = MapEvidence(3, 3, 0.02, (0, 0, 0), [0] * 9, "occupancy_cell")
    assert near.cell(-0.1, 0) == 0 and normal.cell(-0.1, 0) == -1
    import pytest

    with pytest.raises(ValueError, match="ignores rotated"):
        MapEvidence(1, 1, 0.02, (0, 0, 0.1), [100])


def blind_ring(origin_occupied=False, unknown_origin=False):
    grid = rings()
    for iy in range(grid.height):
        for ix in range(grid.width):
            radius = math.hypot(-2.5 + ix * 0.01, -2.5 + iy * 0.01)
            if radius < 0.15:
                grid.data[iy * grid.width + ix] = (
                    100
                    if origin_occupied
                    else -1 if unknown_origin and radius < 0.08 else 100 if radius >= 0.08 else 0
                )
    return grid


def test_blind_inf_has_no_positive_support_and_finite_cannot_see_through_near_wall():
    grid = blind_ring()
    v = score(grid, [math.inf] * 90)
    assert not v["valid"] and v["hits"] == 0 and v["unobserved"] == 90
    assert v["compared"] == 0
    for rays in ([2.0] * 90, [2.0] * 45 + [math.inf] * 45):
        v = score(grid, rays)
        assert not v["valid"] and v["contradiction_fraction"] == 1.0
        assert v["unobserved"] == sum(r == math.inf for r in rays)


def test_occupied_origin_and_unknown_before_near_wall_cannot_support_wrong_pose():
    for rays in ([2.0] * 90, [2.0] * 45 + [math.inf] * 45):
        v = score(blind_ring(origin_occupied=True), rays)
        assert not v["valid"] and v["reason"] == "laser_origin_occupied"
        v = score(blind_ring(unknown_origin=True), rays)
        assert not v["valid"] and v["contradiction_fraction"] == 1.0


def test_excused_inf_cannot_dilute_contradictions_from_observed_beams():
    grid = rings()

    for i in list(range(12)) + list(range(45, 89)):
        angle = -math.pi + i * 2 * math.pi / 90
        gx, gy = grid.grid_point(0.28 * math.cos(angle), 0.28 * math.sin(angle))
        ix, iy = math.floor(gx + 0.5), math.floor(gy + 0.5)
        grid.data[iy * grid.width + ix] = 100
    v = score(grid, [2.0] * 45 + [math.inf] * 45)
    assert v["hits"] == 45 and v["hit_fraction"] == 1.0 and v["known_fraction"] == 1.0
    assert v["unobserved"] == 45 and v["compared"] == 45
    assert math.isclose(v["contradiction_fraction"], 12 / 45)
    assert not v["valid"]


def test_unknown_sensor_values_and_policy_clipped_finite_returns_are_distinct():
    grid = rings()
    v = score(grid, [2.0] * 45 + [9.0] * 45)
    assert v["valid"] and v["hits"] == 45 and v["unobserved"] == 45 and v["compared"] == 45
    assert v["contradictions"] == 0
    for rays in ([math.inf] * 90, [9.0] * 90):
        v = score(grid, rays)
        assert not v["valid"] and v["compared"] == 0 and v["ray_work"] == 0
        assert v["hits"] == 0 and v["contradictions"] == 0

    v = grid.score(
        [2.0] * 45 + [9.0] * 45,
        -math.pi,
        2 * math.pi / 90,
        0.3,
        10.0,
        (0, 0, 0),
        ScorePolicy(max_ray_cells=100000),
    )
    assert not v["valid"] and v["hits"] == 45 and v["unobserved"] == 0
    assert v["policy_clipped"] == 45 and v["compared"] == 90 and v["contradiction_fraction"] > 0.4

    v = grid.score(
        [2.0] * 45 + [10.0] * 45,
        -math.pi,
        2 * math.pi / 90,
        0.3,
        10.0,
        (0, 0, 0),
        ScorePolicy(max_ray_cells=100000),
    )
    assert not v["valid"] and v["policy_clipped"] == 45 and v["unobserved"] == 0
    v = blind_ring().score(
        [9.0] * 90,
        -math.pi,
        2 * math.pi / 90,
        0.3,
        100.0,
        (0, 0, 0),
        ScorePolicy(max_ray_cells=100000),
    )
    assert not v["valid"] and v["policy_clipped"] == 90 and v["contradiction_fraction"] == 1.0


def test_unknown_beams_do_not_refill_uniform_subset_or_support_wrong_pose():
    grid = rings()
    rays = [2.0] * 540
    for i in range(0, 540, 6):
        rays[i] = math.inf
    v = score(grid, rays, increment=2 * math.pi / 540)
    assert not v["valid"] and v["hits"] == 0 and v["compared"] == 0
    for pose in ((1.0, 0, 0), (0, 1.0, 0), (0.5, 0.5, 0)):
        v = score(grid, [2.0] * 45 + [math.inf] * 45, pose)
        assert not v["valid"]
    assert not score(grid, [2.0] * 44 + [math.inf] * 46)["valid"]


def test_blind_blocker_requires_positive_cell_interior_not_boundary_contact():
    for convention, origin in [("occupancy_cell", (0, 0, 0)), ("amcl_nearest", (0.5, 0.5, 0))]:
        grid = MapEvidence(5, 2, 1, origin, [100, 0, 100, 0, 0] + [0] * 5, convention)
        values = list(grid.ray_cells(2, 0.5, -1, 0, 0, 0.25, interior=True))
        assert 100 not in values and 0 in values

        grid = MapEvidence(5, 2, 1, origin, [0] * 5 + [0, 100, 0, 0, 0], convention)
        assert 100 not in grid.ray_cells(0.1, 1, 1, 0, 0, 1.5, interior=True)

        grid = MapEvidence(3, 3, 1, origin, [0, 100, 0, 100, 0, 0, 0, 0, 0], convention)
        assert 100 not in grid.ray_cells(0.5, 0.5, 1, 1, 0, 1, interior=True)


def test_blind_blocker_boundary_unknown_and_rotated_cell_convention():
    for convention, origin in [("occupancy_cell", (0, 0, 0)), ("amcl_nearest", (0.5, 0.5, 0))]:
        grid = MapEvidence(5, 1, 1, origin, [0, 100, 0, 0, 0], convention)
        assert 100 not in grid.ray_cells(0, 0.5, 1, 0, 0, 1, interior=True)
        assert 100 in grid.ray_cells(0, 0.5, 1, 0, 0, 1.01, interior=True)
        unknown = MapEvidence(5, 1, 1, origin, [-1] * 5, convention)
        assert 100 not in unknown.ray_cells(0, 0.5, 1, 0, 0, 0.3, interior=True)
        assert 100 not in grid.ray_cells(-2, 0.5, 1, 0, 0, 0.3, interior=True)
        assert list(grid.ray_cells(0, 0.5, 1, 0, 0, 0, interior=True)) == []

    rotated = MapEvidence(5, 1, 1, (2, 3, math.pi / 2), [0, 100, 0, 0, 0], "occupancy_cell")
    assert 100 not in rotated.ray_cells(1.5, 3, 0, 1, 0, 1, interior=True)
    assert 100 in rotated.ray_cells(1.5, 3, 0, 1, 0, 1.01, interior=True)


def test_new_blind_prefix_work_is_bounded_and_cannot_count_as_known_support():
    grid = rings()
    v = grid.score(
        [2.0] * 90, -math.pi, 2 * math.pi / 90, 0.3, 8.0, (0, 0, 0), ScorePolicy(max_ray_cells=5)
    )
    assert not v["valid"] and v["reason"] == "map_ray_work_budget_exceeded"

    assert not score(blind_ring(unknown_origin=True), [math.inf] * 90)["valid"]
    assert not score(blind_ring(), [2.0] * 44 + [math.inf] * 46)["valid"]


def test_nonprogressing_dda_cannot_bypass_work_budget():

    grid = MapEvidence(3, 1, 1e-18, (0, 0, 0), [0, 100, 0])
    v = grid.score(
        [2.0] * 90, 0, 2 * math.pi / 90, 0.3, 8.0, (0, 0, 0), ScorePolicy(max_ray_cells=5)
    )
    assert not v["valid"] and v["reason"] == "map_ray_work_budget_exceeded"


def test_finite_near_endpoint_keeps_existing_ray_margin():
    grid = rings()
    for i in range(15):
        angle = -math.pi + i * 2 * math.pi / 90
        gx, gy = grid.grid_point(0.31 * math.cos(angle), 0.31 * math.sin(angle))
        grid.data[math.floor(gy + 0.5) * grid.width + math.floor(gx + 0.5)] = 100
    v = score(grid, [0.31] * 15 + [2.0] * 75)
    assert v["valid"] and v["contradiction_fraction"] == 0.0


def test_n10_540_bin_narrow_corridor_uses_only_observable_evidence():

    size_x, size_y, res = 621, 521, 0.01
    origin = (-3.1, -2.6, 0)
    boxes = [
        (-3.1, -3.0, -3.0, 3.0),
        (3.0, 3.1, -3.0, 3.0),
        (-3.0, 3.0, -2.6, -2.5),
        (-3.0, 3.0, 2.5, 2.6),
        (-0.3, 0.3, 0.25, 2.5),
        (-0.3, 0.3, -2.5, -0.25),
    ]
    data = []
    for iy in range(size_y):
        for ix in range(size_x):
            x, y = origin[0] + ix * res, origin[1] + iy * res
            data.append(
                100 if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in boxes) else 0
            )
    grid = MapEvidence(size_x, size_y, res, origin, data)
    pose = (0.0924721442817, -8.97794961929e-6, 0)
    values = []
    for i in range(540):
        a = -math.pi + i * 2 * math.pi / 540
        dx, dy = math.cos(a), math.sin(a)
        distances = []
        for x0, x1, y0, y1 in boxes:
            low, high = 0.0, math.inf
            for p, v, lo, hi in ((pose[0], dx, x0, x1), (pose[1], dy, y0, y1)):
                if abs(v) < 1e-15:
                    if not lo <= p <= hi:
                        high = -1
                else:
                    near, far = sorted(((lo - p) / v, (hi - p) / v))
                    low = max(low, near)
                    high = min(high, far)
            if low <= high and high >= 0:
                distances.append(low)
        r = min(distances)
        values.append(r if r >= 0.3 else math.inf)
    v = score(grid, values, pose, increment=2 * math.pi / 540)
    assert v["valid"] and v["hits"] >= 45 and v["unobserved"] >= 20
    assert v["hit_fraction"] == 1.0 and v["known_fraction"] == 1.0
    assert v["compared"] == 90 - v["unobserved"]
    for wrong_y in (-0.5, 0.5):
        v = score(grid, values, (pose[0], wrong_y, 0), increment=2 * math.pi / 540)
        assert not v["valid"] and v["reason"] == "laser_origin_occupied"


def monitor_fixture():
    import pytest

    pytest.importorskip("rclpy")
    from types import SimpleNamespace as S
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import LaserScan
    from duojin02_bringup.localization_monitor import LocalizationMonitor

    tf = TransformStamped()
    tf.transform.rotation.w = 1.0
    reports = []
    monitor = S(
        policy=ScorePolicy(),
        pending=None,
        received=0,
        processed=0,
        map_frame="map",
        convention="amcl_nearest",
        grid=None,
        map_signature=None,
        buffer=S(lookup_transform=lambda *_: tf),
        publish=lambda *args: reports.append(args),
    )
    msg = LaserScan()
    msg.header.frame_id = "laser"
    msg.header.stamp.sec = 1
    msg.angle_min, msg.angle_increment = -math.pi, 2 * math.pi / 90
    msg.angle_max = msg.angle_min + 89 * msg.angle_increment
    msg.range_min, msg.range_max = 0.3, 8.0
    msg.ranges = [2.0] * 90
    return LocalizationMonitor, monitor, msg, reports


def test_quality_mismatch_is_not_latched_and_next_good_scan_reports_immediately():
    from types import SimpleNamespace as S

    methods, monitor, msg, reports = monitor_fixture()
    values = iter(
        [
            {"valid": True, "reason": "consistent"},
            {"valid": False, "reason": "scan_map_inconsistent_or_occluded"},
            {"valid": True, "reason": "consistent"},
        ]
    )
    monitor.grid = S(score=lambda *_: next(values))
    for stamp in (1, 2, 3):
        msg.header.stamp.sec = stamp
        methods.on_scan(monitor, msg)
        methods.tick(monitor)
    assert [args[1]["valid"] for args in reports] == [True, False, True]
    assert monitor.received == monitor.processed == 3
    assert monitor.pending is None


def test_unavailable_exact_transform_drops_one_diagnostic_and_accepts_next_scan():
    from types import SimpleNamespace as S

    methods, monitor, msg, reports = monitor_fixture()
    from tf2_ros import TransformException

    monitor.grid = S(score=lambda *_: {"valid": True, "reason": "consistent"})
    ready = monitor.buffer

    def missing(*_):
        raise TransformException("Exact transform missing")

    monitor.buffer = S(lookup_transform=missing)
    methods.on_scan(monitor, msg)
    methods.tick(monitor)
    assert monitor.pending is None and reports == [("exact_transform_unavailable",)]
    monitor.buffer = ready
    msg.header.stamp.sec = 2
    methods.on_scan(monitor, msg)
    methods.tick(monitor)
    assert reports[-1][1]["valid"] is True and monitor.processed == 1


def test_invalid_map_can_be_replaced_without_a_recovery_service():
    methods, monitor, _msg, reports = monitor_fixture()
    from nav_msgs.msg import OccupancyGrid

    valid = OccupancyGrid()
    valid.header.frame_id = "map"
    valid.info.width = valid.info.height = 2
    valid.info.resolution = 0.01
    valid.info.origin.orientation.w = 1.0
    valid.data = [0, 0, 0, 100]
    methods.on_map(monitor, valid)
    original = monitor.grid
    assert original is not None
    methods.on_map(monitor, valid)
    assert len(reports) == 1
    valid.data = [0]
    methods.on_map(monitor, valid)
    assert monitor.grid is None and monitor.map_signature is None
    valid.data = [0, 0, 0, 100]
    methods.on_map(monitor, valid)
    assert monitor.grid is not None and monitor.grid is not original
    assert reports[-1] == ("map_received_or_changed",)


def test_status_marks_quality_as_advisory_and_warns_without_motion_interface():
    import json
    from types import SimpleNamespace as S

    methods, monitor, _msg, _reports = monitor_fixture()
    published, warnings = [], []
    monitor.status_pub = S(publish=lambda message: published.append(json.loads(message.data)))
    monitor.get_clock = lambda: S(now=lambda: S(nanoseconds=10_000_000_000))
    monitor.get_logger = lambda: S(warning=lambda *args, **kwargs: warnings.append((args, kwargs)))
    methods.publish(monitor, "scan_map_inconsistent_or_occluded", {"valid": False}, 9.9)
    assert published[-1]["advisory_only"] is True
    assert published[-1]["quality"] == "inconsistent" and published[-1]["consistent"] is False
    assert warnings[-1][1]["throttle_duration_sec"] == 2.0
    methods.publish(monitor, "consistent", {"valid": True}, 10.0)
    assert published[-1]["quality"] == "consistent" and len(warnings) == 1
