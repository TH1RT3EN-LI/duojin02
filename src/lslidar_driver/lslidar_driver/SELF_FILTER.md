> 当前配置与启动方式已收敛至 `duojin01_bringup/config/chassis.yaml`；本文原参数文件名属于历史记录，当前唯一来源见 [配置说明](../../docs/chassis/configuration.md)。

# N10Plus chassis projection crop

The `min_range` / `max_range` gate is a sensor range contract, independent of
chassis geometry. The N10Plus driver and hardware profile now use a 0.02 m floor,
replacing the old 0.30 m circle that discarded external near returns. The generic
geometry API remains opt-in; `config/duojin01_n10plus.yaml` explicitly enables
the chassis crop and the unmasked diagnostic topics for this robot.

When enabled, it evaluates **each available echo before nearest-range reduction**.
The driver transforms its laser-frame XYZ coordinates with
`Rz(yaw) * Ry(pitch) * Rx(roll)` and translation into the configured body frame.
A return is removed only if it lies strictly inside the supplied simple polygon
and finite height interval, farther than the ambiguity band from each surface.
Concave simple polygons and either winding are supported. Holes, multiple
disconnected prisms, moving parts, and automatic TF lookup are not implemented.

There are two distinct uses of the supplied prism. A material self-filter needs
verified material intersected by the scan plane and calibrated mounting pose.
The current hardware profile instead implements the user's explicit **chassis
projection exclusion policy**: crop the strict interior of the unpadded chassis
projection, removing every strict interior point and preserving every point outside it. Inclusion
does not prove physical self identity; empty space inside the projection is also
excluded by this policy. It does not automatically load navigation inflation or
a collision box. Arms and the upper body are not part of the mask.

The hardware profile uses the current model's laser-to-`base_link` XYZ and zero
RPY. Physical heading correction is explicitly deferred by the user, and the
130.233 mm height remains the current CAD proxy rather than a new measurement.
The active hardware profile uses `boundary_ambiguity_m: 0.0`: there is no
physical inner retention band. The previous 0.03 m band leaked rare interior
returns into obstacle memory and conflicted with the user's projection policy.
Only exact surfaces and floating-point boundary uncertainty remain observable.
The generic API still supports a nonzero band as an explicit alternative
policy; it shrinks deletion and never dilates it outside the supplied prism.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `self_filter.enabled` | `false` | Enable the fixed geometry mask; hardware YAML explicitly sets `true`. |
| `self_filter.laser_to_body_xyz_m` | `[]` | Explicit laser origin XYZ in the model's body frame, metres. |
| `self_filter.laser_to_body_rpy_rad` | `[]` | Explicit roll, pitch, yaw in radians; rotation order as above. |
| `self_filter.body_polygon_xy_m` | `[]` | Flat `[x0,y0,x1,y1,...]` simple polygon; omit a repeated closing vertex. |
| `self_filter.body_z_bounds_m` | `[]` | Explicit `[lower_z, upper_z]`, metres, with `lower_z < upper_z`. |
| `self_filter.boundary_ambiguity_m` | `0.0` | Preserve points this close to either side of the surfaces. Never dilates the deletion mask. |
| `self_filter.publish_raw` | `false` | Publish range-gated, unmasked diagnostics alongside normal filtered topics. |
| `self_filter.raw_scan_topic` | `/scan_raw` | Unmasked nearest-return LaserScan. |
| `self_filter.raw_pointcloud_topic` | `/lslidar_point_cloud_raw` | Unmasked PointCloud2 retaining both available echoes. |

Enabling the filter without explicit finite XYZ, RPY, polygon, or height bounds
fails during driver construction, before serial acquisition starts. Invalid
height order, negative/nonfinite ambiguity, duplicate or overlapping vertices,
degenerate area, and polygon self-intersections are rejected. Parameters are
fixed at construction; changing them requires restarting this driver.
When raw publishing is enabled, raw destinations must be distinct from filtered
destinations and from each other after ROS namespace expansion and remapping.

Normal point-cloud output retains all unmasked echoes in the **original laser
frame**. Normal LaserScan chooses the nearest unmasked echo per angular bin. If
an angular bin contained masked returns and contains no retained return, its
range becomes `NaN` (unknown), never an invented free-space observation.
Untouched empty bins keep their legacy `+Inf` behavior. Numerically ambiguous
surface points remain available. A nonzero opt-in band would also retain its
points, but the active chassis profile has no physical band.

Raw and filtered outputs share the acquisition frame's timestamp, scan period,
angle convention, and frame ID. Raw PointCloud2 is captured before self masking,
but **after the original sensor range gate**: it cannot recover any echo rejected
by `min_range` / `max_range`. Raw LaserScan also reduces two echoes to one range;
use the raw point cloud when inspecting alternative echoes. Enable raw topics
only when their extra bandwidth is needed, and check consumers treat `NaN` as
unknown before enabling a mask in a navigation deployment.

AMCL and SLAM Toolbox hardware profiles also use the 0.02 m floor. Localization
health checks skip `NaN` unknown directions while retaining their fixed angular
sample roster, minimum valid-hit requirements and malformed-scan rejection.
Runtime geometry parameters are fixed at startup. Rebuild/reinstall and restart
the driver to use edited source profiles; editing this file does not change a
running device. Disable `self_filter.enabled` to compare unmasked behavior; raw
topics already provide the corresponding range-gated diagnostic observations.

`include/lslidar_driver/self_filter.hpp` is a standalone C++17 geometry API for
offline replay. The test prisms are synthetic geometry fixtures, not calibrated
robot material. `test_self_filter` covers offset noncircular shapes, concavity,
full rigid transforms, finite height, surface ambiguity, and invalid models;
`test_x10_driver` covers echo selection, unknown bins, preservation of raw data,
matching output timestamps, and legacy behavior while disabled.

Current strict-policy validation: 28 native tests and 41 simulation/geometry tests pass; real packet publication replay preserves all 323 saved external near echoes, and fresh Jetson corner60 seeds21/22 both arrive with zero collisions. The physical driver was not started. See [current correction evidence](/home/th1rt3en/Documents/Codex/2026-10-13/outputs/chassis-projection-crop-correction-20261013/report.md).
