#include "duojin02_navigation/tracking.hpp"
#include "duojin02_navigation/path_cleanup.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace duojin02_navigation;

// Frozen pre-batch implementation: an independent regression oracle for the
// reaction boundary, final floating-point time sample and pruning padding.
bool originalBrakingSweep(Pose pose, Velocity v, const Polygon &polygon,
                          const std::vector<Point> &points) {
  constexpr double margin = .010, reaction = .2, deceleration = .4,
                   angular_deceleration = .8, spatial_step = .004;
  double radius = 0;
  for (const auto &vertex : polygon)
    radius = std::max(radius, std::hypot(vertex.x, vertex.y));
  const double speed = std::hypot(v.x, v.y), corner_speed = speed + radius * std::abs(v.yaw);
  const double dt = std::min(.02, spatial_step / std::max(corner_speed, .001));
  const double end = reaction + std::max(speed / deceleration,
                                        std::abs(v.yaw) / angular_deceleration) + dt;
  const double sampling_margin = corner_speed * dt / 2;
  std::vector<Pose> poses;
  double min_x = 1e100, min_y = 1e100, max_x = -1e100, max_y = -1e100;
  for (double t = 0; t <= end; t += dt) {
    poses.push_back(pose);
    for (const auto &vertex : polygon) {
      const auto p = transform(vertex, pose);
      min_x = std::min(min_x, p.x);
      min_y = std::min(min_y, p.y);
      max_x = std::max(max_x, p.x);
      max_y = std::max(max_y, p.y);
    }
    pose = advance(pose, v, dt);
    if (t >= reaction) {
      const double current = std::hypot(v.x, v.y);
      const double factor = current > 0 ? std::max(0., 1 - deceleration * dt / current) : 0;
      v.x *= factor;
      v.y *= factor;
      v.yaw = approachZero(v.yaw, angular_deceleration * dt);
    }
  }
  std::vector<Point> nearby;
  for (const auto &p : points) {
    const double padding = margin + sampling_margin + p.radius;
    if (p.x >= min_x - padding && p.x <= max_x + padding && p.y >= min_y - padding &&
        p.y <= max_y + padding)
      nearby.push_back(p);
  }
  for (const auto &sample : poses)
    if (collision(sample, polygon, nearby, margin + sampling_margin))
      return true;
  return false;
}

int main() {
  std::vector<Pose> repeated{{0, 0, 0},   {.06, 0, 0}, {.12, 0, 0},
                             {.06, 0, 0}, {.12, 0, 0}, {.18, 0, 0}};
  assert((loopFreeIndices(repeated) == std::vector<size_t>{0, 3, 4, 5}));
  std::vector<Pose> turns{{0, 0, 0}, {0, 0, M_PI / 2}, {.1, 0, M_PI / 2}, {.1, 0, 0}};
  assert((loopFreeIndices(turns) == std::vector<size_t>{0, 1, 2, 3}));
  std::vector<Pose> nested{{0, 0, 0}, {.1, 0, 0}, {.2, 0, 0}, {.1, 0, 0}, {0, 0, 0}, {.3, 0, 0}};
  assert((loopFreeIndices(nested) == std::vector<size_t>{4, 5}));
  Polygon p{{.221, .199465},   {.209465, .211},   {-.210879, .211}, {-.222, .199879},
            {-.222, -.198465}, {-.209465, -.211}, {.210879, -.211}, {.221, -.200879}};
  std::mt19937 rng(0xD021);
  std::uniform_real_distribution<double> location(-.6, .6), rotation(-3.14, 3.14), radius(0, .10);
  for (int i = 0; i < 10000; ++i) {
    const Pose pose{location(rng), location(rng), rotation(rng)};
    const Point q{location(rng), location(rng), radius(rng)};
    const Point world = transform(q, pose);
    assert(collision(pose, p, {world}, .010) == touches(q, p, .010));
  }
  std::vector<Point> walls;
  for (int i = -50; i <= 50; ++i) {
    walls.push_back({i * .01, .25, .007071});
    walls.push_back({i * .01, -.242, .007071});
  }
  const double biased = footprintClearance({}, p, walls),
               centered = footprintClearance({0, .004, 0}, p, walls);
  assert(centered > biased + .0035);
  assert(!collision({0, .004, 0}, p, walls, .010));
  assert(footprintClearance({}, p, {{.1, 0, .007}}) < 0);
  for (int i = 0; i <= 25; ++i)
    assert(!collision({i * .004, 0, 0}, p, {{.36, 0, .007}}, .010));
  assert(brakingSweep({}, {.5, 0, 0}, p, {{.36, 0, .007}}, .010, .2, .4, .8));
  assert(!brakingSweep({}, {.04, 0, 0}, p, {{.36, 0, .007}}, .010, .2, .4, .8));
  for (int i = 0; i <= 25; ++i)
    assert(!collision({i * .004, 0, i * .03 / 25}, p, {{-.18, -.24, .007}}, .010));
  assert(brakingSweep({}, {0, 0, .45}, p, {{-.18, -.24, .007}}, .010, .2, .4, .8));

  // Flattened CUDA batches use the same variable margin at every pose of a
  // sweep. Compare that full-scene evaluation with both the frozen CPU oracle
  // and the shared CPU implementation, including spatial-pruning boundaries.
  std::uniform_real_distribution<double> velocity(-.65, .65);
  for (int trial = 0; trial < 300; ++trial) {
    const Pose start{location(rng), location(rng), rotation(rng)};
    const Velocity command{velocity(rng), velocity(rng), 2 * velocity(rng)};
    std::vector<Point> scene;
    for (int j = 0; j < 24; ++j)
      scene.push_back({location(rng), location(rng), .05 * radius(rng)});
    if (trial % 4 == 0)
      scene.clear();
    const auto samples = brakingSweepSamples(start, command, p, .010, .2, .4, .8);
    assert(!samples.poses.empty() && samples.margin >= .010 && samples.margin <= .012000001);
    const bool flat_hit = std::any_of(samples.poses.begin(), samples.poses.end(),
                                     [&](const Pose &q) { return collision(q, p, scene, samples.margin); });
    const bool oracle = originalBrakingSweep(start, command, p, scene);
    assert(flat_hit == oracle);
    assert(brakingSweep(start, command, p, scene, .010, .2, .4, .8) == oracle);
  }
  // The whole-sweep world gate is semantically significant for the original
  // -1e-10 inside tolerance on a thin convex footprint. A flattened GPU query
  // must carry this gate; merely checking every pose against the full scene
  // would reject an earlier scale that the frozen CPU implementation admits.
  const Polygon thin{{0, 0}, {.5000000001, .5}, {.5, .5}};
  const Pose thin_start{0, 0, -M_PI / 4};
  const Point beyond_world_gate = transform({.509, .509}, thin_start);
  assert(collision(thin_start, thin, {beyond_world_gate}, .010));
  assert(!originalBrakingSweep(thin_start, {}, thin, {beyond_world_gate}));
  assert(!brakingSweep(thin_start, {}, thin, {beyond_world_gate}, .010, .2, .4, .8));
  const auto thin_samples = brakingSweepSamples(thin_start, {}, thin, .010, .2, .4, .8);
  assert(beyond_world_gate.x > thin_samples.max_x + thin_samples.margin);
  const auto candidates = centeringCandidates({1, 2, .3}, {1.3, 2.4, .7}, .65);
  constexpr std::array<double, 7> expected_offsets{0, -.01, .01, -.02, .02, -.03, .03};
  for (size_t i = 0; i < candidates.size(); ++i) {
    assert(candidates[i].offset == expected_offsets[i]);
    assert(std::abs(candidates[i].pose.x - (1 + .65 * .3 - .8 * expected_offsets[i])) < 1e-12);
    assert(std::abs(candidates[i].pose.y - (2 + .65 * .4 + .6 * expected_offsets[i])) < 1e-12);
    assert(candidates[i].pose.yaw == .7);
  }
  const auto transitions = brakingCandidateVelocities({.4, -.2, .1}, {-.2, .3, -.5});
  assert(transitions.front().x == .4 && transitions.front().y == -.2);
  for (size_t i = 1; i < transitions.size(); ++i) {
    const double fraction = .25 * i;
    assert(transitions[i].x == -.2 + fraction * (.4 - -.2));
    assert(transitions[i].y == .3 + fraction * (-.2 - .3));
    assert(transitions[i].yaw == -.5 + fraction * (.1 - -.5));
  }
  ControllerGeometryStatistics timing;
  for (int i = 0; i < 99; ++i)
    timing.recordCycle(1.1);
  timing.recordCycle(400.);
  assert(timing.cycles == 100 && timing.cycleQuantileUpperBound(.99) == 1.25);
  assert(timing.cycleQuantileUpperBound(1.) == 400.);

  double footprint_radius = 0;
  for (const auto &q : p)
    footprint_radius = std::max(footprint_radius, std::hypot(q.x, q.y));
  const double old_forward_only = footprint_radius + .45 + .014 + .01 * std::sqrt(2.0) / 2;
  bool counterexample = false;
  for (double vx : {-.2, .0, .5, .75})
    for (double vy : {-.3, .0, .3, .5})
      for (double omega : {-.65, .0, .65, 2.}) {
        Velocity velocity{vx, vy, omega};
        Pose pose;
        const double extent = localSnapshotExtent(p, .5, .2, .3, velocity, .01);
        const double stop = .2 + std::max(std::hypot(vx, vy) / .4, std::abs(omega) / .8) + .01;
        for (double t = 0; t <= stop; t += .001) {
          for (const auto &vertex : p) {
            const auto point = transform(vertex, pose);
            assert(std::abs(point.x) + .012 + .01 * std::sqrt(2.0) / 2 <= extent);
            assert(std::abs(point.y) + .012 + .01 * std::sqrt(2.0) / 2 <= extent);
            if (vx == .5 && std::abs(vy) == .3 && std::abs(omega) == .65 &&
                std::abs(point.x) + .012 + .01 * std::sqrt(2.0) / 2 > old_forward_only)
              counterexample = true;
          }
          pose = advance(pose, velocity, .001);
          if (t >= .2) {
            double speed = std::hypot(velocity.x, velocity.y);
            double factor = speed > 0 ? std::max(0., 1 - .4 * .001 / speed) : 0;
            velocity.x *= factor;
            velocity.y *= factor;
            velocity.yaw = approachZero(velocity.yaw, .8 * .001);
          }
        }
      }
  assert(counterexample);
  std::cout
      << "10000 unpruned broad-phase comparisons; 300 frozen braking oracle / full-scene batch comparisons; centered candidate and transition order; bounded timing; noisy corridor balance; thin interior; safe chord versus unsafe speed and angular-rate sweeps passed\n";
}
