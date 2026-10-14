#!/usr/bin/env python3
"""Exercise the production C++ mask on frozen scan geometry (no ROS or robot).

The inherited XY footprint is a surrogate for reproducing the old fixture,
not a calibrated optical chassis model. The capture has one return per beam;
dual-echo preservation is tested separately with protocol-level driver tests.
"""
import argparse
import csv
import hashlib
import json
import math
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
INCLUDE = ROOT / "src/lslidar_driver/lslidar_driver/include"
RUNNER = r'''
#include "lslidar_driver/self_filter.hpp"
#include <chrono>
#include <iostream>
int main() {
  using namespace lslidar_driver;
  size_t n, count; double band;
  std::cin >> n >> count >> band;
  SelfFilterConfig config; config.enabled = true;
  config.boundary_ambiguity_m = band;
  for (size_t i = 0; i < 3; ++i) {
    double v; std::cin >> v; config.laser_to_body_xyz_m.push_back(v);
  }
  config.laser_to_body_rpy_rad = {0.0, 0.0, 0.0};
  // A deliberately declared finite prism reproduces only the old XY mask.
  config.body_z_bounds_m = {-1.0, 1.0};
  for (size_t i = 0; i < n * 2; ++i) {
    double v; std::cin >> v; config.body_polygon_xy_m.push_back(v);
  }
  const SelfFilter filter(config);
  std::vector<Point3> points(count);
  for (auto &p : points) std::cin >> p.x >> p.y >> p.z;
  if (!std::cin) return 2;
  std::vector<SelfFilterClassification> labels; labels.reserve(count);
  const auto start = std::chrono::steady_clock::now();
  for (const auto &p : points) labels.push_back(filter.classify(p));
  const auto stop = std::chrono::steady_clock::now();
  for (const auto label : labels)
    std::cout << (label == SelfFilterClassification::Inside ? 'I' :
                  label == SelfFilterClassification::Ambiguous ? 'A' : 'O') << '\n';
  std::cerr << std::chrono::duration<double, std::micro>(stop-start).count() << '\n';
}
'''


def inside_convex(x, y, polygon):
    """Independent oracle matching the existing frozen convex fixture."""
    return all((b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]) >= -1e-10
               for a, b in zip(polygon, polygon[1:]+polygon[:1]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("geometry", type=Path,
                        help="previous lidar-range-counterfactual.json")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    geometry = json.loads(args.geometry.read_text())
    polygon, origin = geometry["polygon"], geometry["laser_xy"]
    pairs = {}
    for line in args.capture.open():
        row = json.loads(line)
        pairs.setdefault(row["source_stamp_ns"], {})[row["topic"]] = row
    points, info, scans = [], [], {}
    for stamp, pair in sorted(pairs.items()):
        raw = pair["/sim/lidar/raw"]
        scans[stamp] = {"source_stamp_s": stamp/1e9,
                        "selected_0_30": 0, "selected_0_15_band0": 0,
                        "selected_0_15_band0_03": 0}
        stride = max(1, math.ceil(len(raw["ranges"])/90))
        for index, value in enumerate(raw["ranges"]):
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                continue
            angle = raw["angle_min"]+index*raw["angle_increment"]
            x, y = value*math.cos(angle), value*math.sin(angle)
            inside = inside_convex(x+origin[0], y+origin[1], polygon)
            points.append((x, y, 0.0))
            info.append((stamp, index % stride == 0, value, inside))
            if index % stride == 0 and .3 <= value < 8 and not inside:
                scans[stamp]["selected_0_30"] += 1
    totals = {}
    with tempfile.TemporaryDirectory(prefix="lidar-self-mask-") as temporary:
        source, executable = Path(temporary)/"runner.cpp", Path(temporary)/"runner"
        source.write_text(RUNNER)
        subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra",
                        "-I", str(INCLUDE), str(source), "-o", str(executable)],
                       check=True)
        for band, field in [(0., "selected_0_15_band0"),
                            (.03, "selected_0_15_band0_03")]:
            # Z is immaterial for this explicitly declared surrogate prism.
            words = [len(polygon), len(points), band, *origin, 0.14330766582,
                     *(v for vertex in polygon for v in vertex)]
            payload = " ".join(map(str, words))+"\n"+"\n".join(
                " ".join(map(str, point)) for point in points)+"\n"
            run = subprocess.run([str(executable)], input=payload,
                                 text=True, capture_output=True, check=True)
            labels = run.stdout.splitlines()
            assert len(labels) == len(info)
            outside_removed, exterior_kept, ambiguous, inside_removed = 0, 0, 0, 0
            recovered_near = 0
            for (stamp, selected, value, inside), label in zip(info, labels):
                eligible = .15 <= value < 8
                if eligible and not inside:
                    outside_removed += label == "I"
                    exterior_kept += label != "I"
                    recovered_near += value < .3 and label != "I"
                ambiguous += label == "A"
                inside_removed += label == "I"
                if selected and eligible and label != "I":
                    scans[stamp][field] += 1
            assert outside_removed == 0
            totals[str(band)] = {"classified_finite_points": len(info),
                "eligible_outside_surrogate_points_removed": outside_removed,
                "eligible_outside_surrogate_points_kept": exterior_kept,
                "outside_surrogate_near_points_recovered": recovered_near,
                "ambiguous_points_kept": ambiguous,
                "inside_surrogate_points_removed": inside_removed,
                "desktop_classification_us_per_540_points":
                    float(run.stderr.strip())/len(info)*540}
    rows = list(scans.values())
    for field in ["selected_0_30", "selected_0_15_band0", "selected_0_15_band0_03"]:
        values = [row[field] for row in rows]
        totals[field] = {"min": min(values), "max": max(values),
                         "frames_below45": sum(v < 45 for v in values)}
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output/"replay.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    summary = {"capture": str(args.capture),
        "capture_sha256": hashlib.sha256(args.capture.read_bytes()).hexdigest(),
        "header_sha256": hashlib.sha256((INCLUDE/"lslidar_driver/self_filter.hpp").read_bytes()).hexdigest(),
        "scan_count": len(rows), "totals": totals,
        "critical_scan": next(row for row in rows if row["source_stamp_s"] == 16.1),
        "limitations": ["Inherited footprint is an optical-mask surrogate, not real self/external ground truth.",
            "The 0.15 range floor and 0.03 ambiguity band are explicit experimental values, not calibrated hardware settings.",
            "No raw dual echoes exist in this capture; driver protocol tests cover second-echo retention.",
            "Timing is a desktop microbenchmark; no Jetson or closed-loop accuracy claim."]}
    (args.output/"replay.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
