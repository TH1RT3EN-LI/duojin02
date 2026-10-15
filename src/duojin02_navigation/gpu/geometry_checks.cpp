#include "duojin02_navigation/gpu_geometry.hpp"
#include "duojin02_navigation/tracking.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <random>

using namespace duojin02_navigation;

template <class F> void rejects(F &&f) {
  bool failed = false;
  try {
    f();
  } catch (const std::exception &) {
    failed = true;
  }
  assert(failed);
}

int main(int argc, char **argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  const bool required = mode == "--require-cuda";
  GeometryCompute cpu({false, 0});
  GeometryCompute automatic({true, 0});
  assert(!cpu.available());
  if (mode == "--no-device")
    assert(!automatic.available());
  if (required && !automatic.available()) {
    std::cout << "CUDA device unavailable: " << automatic.deviceName() << '\n';
    return 77;
  }
  const Polygon polygon{{.213, .191465},   {.201465, .203},   {-.202879, .203}, {-.214, .191879},
                        {-.214, -.190465}, {-.201465, -.203}, {.202879, -.203}, {.213, -.192879}};
  std::mt19937_64 random(20261010);
  std::uniform_real_distribution<double> xy(-3., 3.), angle(-3.14159, 3.14159);
  size_t comparisons = 0;
  size_t clearance_comparisons = 0, variable_margin_comparisons = 0, bounded_comparisons = 0;
  auto compare = [&](const Polygon &shape, const std::vector<Point> &points,
                     const std::vector<Pose> &poses, double margin) {
    cpu.setScene(shape, points);
    automatic.setScene(shape, points);
    const auto a = cpu.evaluate(poses, margin), b = automatic.evaluate(poses, margin);
    assert(a.collisions == b.collisions);
    if (required && !points.empty() && !poses.empty())
      assert(b.used_cuda);
    for (size_t i = 0; i < poses.size(); ++i) {
      assert(static_cast<bool>(a.collisions[i]) == collision(poses[i], shape, points, margin));
      ++comparisons;
    }
    const auto ca = cpu.evaluateWithClearance(poses, margin),
               cb = automatic.evaluateWithClearance(poses, margin);
    assert(ca.collisions == a.collisions && cb.collisions == a.collisions);
    assert(ca.clearances.size() == poses.size() && cb.clearances.size() == poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
      assert(ca.clearances[i] == footprintClearance(poses[i], shape, points));
      assert(std::abs(cb.clearances[i] - ca.clearances[i]) <= cb.clearance_error_bounds[i]);
      ++clearance_comparisons;
    }
  };
  for (int scene = 0; scene < 24; ++scene) {
    std::vector<Point> points;
    std::vector<Pose> poses;
    for (int i = 0; i < 1000; ++i)
      points.push_back({xy(random), xy(random), .007071067812 + (i % 7) * .0005});
    points.push_back({0, 0, .001});
    points.push_back({.223, 0, 0});
    for (int i = 0; i < 192; ++i)
      poses.push_back({xy(random), xy(random), angle(random)});
    poses.push_back({0, 0, 0});
    poses.push_back({.5, 0, 0});
    poses.push_back({0, 0, 1.5707963267948966});
    compare(polygon, points, poses, .010);

    std::vector<double> margins;
    for (size_t i = 0; i < poses.size(); ++i)
      margins.push_back((i % 9) * .003);
    const auto variable_cpu = cpu.evaluate(poses, margins),
               variable_gpu = automatic.evaluate(poses, margins);
    assert(variable_cpu.collisions == variable_gpu.collisions);
    for (size_t i = 0; i < poses.size(); ++i) {
      assert(static_cast<bool>(variable_cpu.collisions[i]) ==
             collision(poses[i], polygon, points, margins[i]));
      ++variable_margin_comparisons;
    }
    std::vector<GeometryWorldBounds> world_bounds;
    for (const auto &p : poses)
      world_bounds.push_back({p.x - .15, p.y - .12, p.x + .17, p.y + .14});
    const auto bounded_cpu = cpu.evaluate(poses, margins, world_bounds),
               bounded_gpu = automatic.evaluate(poses, margins, world_bounds);
    assert(bounded_cpu.collisions == bounded_gpu.collisions);
    for (size_t i = 0; i < poses.size(); ++i) {
      std::vector<Point> filtered;
      const auto &b = world_bounds[i];
      for (const auto &p : points) {
        const double padding = margins[i] + p.radius;
        if (p.x >= b.min_x - padding && p.x <= b.max_x + padding &&
            p.y >= b.min_y - padding && p.y <= b.max_y + padding)
          filtered.push_back(p);
      }
      assert(static_cast<bool>(bounded_cpu.collisions[i]) ==
             collision(poses[i], polygon, filtered, margins[i]));
      ++bounded_comparisons;
    }

    compare(polygon, {{2.9, 2.9, 0}}, poses, .010);
    compare(polygon, {}, poses, .010);
  }
  const Polygon square{{-.2, -.2}, {.2, -.2}, {.2, .2}, {-.2, .2}};

  for (double radius : {0., .007071067812}) {
    const double boundary = .2 + .010 + radius;
    for (double x : {boundary, std::nextafter(boundary, 0.), std::nextafter(boundary, 1.),
                     boundary - 1e-9, boundary + 1e-9}) {
      compare(square, {{x, 0, radius}}, {{0, 0, 0}, {0, 0, 1.5707963267948966}}, .010);
      compare(square, {{x, x, radius}}, {{0, 0, 0}}, .010);
    }
  }
  for (double x :
       {.2, .2 + 2.5e-10, std::nextafter(.2 + 2.5e-10, 0.), std::nextafter(.2 + 2.5e-10, 1.)})
    compare(square, {{x, 0, 0}}, {{0, 0, 0}}, 0.);
  for (double yaw : {0., .1, .5235987755982988, .7, 1.5707963267948966, 2.4, -2.9})
    for (double origin : {0., 1e3, 1e6}) {
      const Pose query{origin, -origin, yaw};
      for (const auto &local : std::vector<Point>{{.21, 0, 0},
                                                  {-.21, 0, 0},
                                                  {0, .21, 0},
                                                  {0, -.21, 0},
                                                  {.20707106781186548, .20707106781186548, 0},
                                                  {-.20707106781186548, -.20707106781186548, 0}}) {
        const auto world = transform(local, query);
        for (double x : {world.x, std::nextafter(world.x, -std::numeric_limits<double>::infinity()),
                         std::nextafter(world.x, std::numeric_limits<double>::infinity())})
          compare(square, {{x, world.y, 0}}, {query}, .010);
      }
    }
  for (double origin : {1e3, 1e6, 1e9, 1e12})
    for (double x : {.21, std::nextafter(.21, 0.), std::nextafter(.21, 1.), .21 - 1e-5, .21 + 1e-5})
      compare(square, {{origin + x, origin, 0}}, {{origin, origin, 0}}, .010);
  compare(square, {{0, 0, 0}}, {}, .010);
  // The trajectory world-AABB broad phase is part of brakingSweep semantics.
  // For a thin valid polygon, the -1e-10 inside tolerance can admit a point
  // rejected by that broad phase; flattening must preserve this distinction.
  const Polygon thin{{0, 0}, {.5000000001, .5}, {.5, .5}};
  const Pose rotated{0, 0, -3.14159265358979323846 / 4};
  const Point thin_point = transform({.509, .509}, rotated);
  GeometryWorldBounds thin_bounds{1e100, 1e100, -1e100, -1e100};
  for (const auto &p : thin) {
    const auto v = transform(p, rotated);
    thin_bounds.min_x = std::min(thin_bounds.min_x, v.x);
    thin_bounds.min_y = std::min(thin_bounds.min_y, v.y);
    thin_bounds.max_x = std::max(thin_bounds.max_x, v.x);
    thin_bounds.max_y = std::max(thin_bounds.max_y, v.y);
  }
  for (auto *backend : {&cpu, &automatic}) {
    backend->setScene(thin, {thin_point});
    assert(backend->evaluate({rotated}, .01).collisions == std::vector<uint8_t>{1});
    assert(backend->evaluate({rotated}, {.01}, {thin_bounds}).collisions == std::vector<uint8_t>{0});
  }
  for (double radius : {0., .007})
    for (double x : {.01 + radius, std::nextafter(.01 + radius, 0.),
                     std::nextafter(.01 + radius, 1.)}) {
      const Point p{x, 0, radius};
      for (auto *backend : {&cpu, &automatic}) {
        backend->setScene(square, {p});
        const auto answer = backend->evaluate({{}}, {.01}, {{0, 0, 0, 0}});
        assert(static_cast<bool>(answer.collisions[0]) == (x <= .01 + radius));
      }
    }
  // Bucket boundaries, far-outside queries, cap boundaries, and varying disc
  // radii must retain the unindexed CPU's complete predicate and signed score.
  for (double origin : {0., 19.375, -22.125}) {
    std::vector<Point> points;
    for (int i = -40; i <= 40; ++i)
      points.push_back({origin + i * .125, (i % 7) * .125, (i + 40) % 5 * .02});
    points.push_back({origin + 4, 0, 4});
    std::vector<Pose> poses;
    for (int i = -32; i <= 32; ++i)
      poses.push_back({origin + i * .125, i % 3 * .07, i * .13});
    poses.push_back({origin - 100, -100, .3});
    poses.push_back({origin + 100, 100, .7});
    compare(square, points, poses, .013);
    for (double cap : {0., .001, .08, .3}) {
      const auto scores = automatic.clearance(poses, cap);
      for (size_t i = 0; i < poses.size(); ++i) {
        assert(std::abs(scores.clearances[i] - footprintClearance(poses[i], square, points, cap)) <=
               scores.clearance_error_bounds[i]);
        ++clearance_comparisons;
      }
    }
  }
  // setScene owns the source storage and detects content changes, even when
  // callers retain the same vector addresses or construct temporary vectors.
  for (bool use_gpu : {false, true}) {
    GeometryCompute owned({use_gpu, 0});
    owned.setScene(Polygon(square), std::vector<Point>{{0, 0, .001}});
    const auto initial = owned.evaluate({{}}, .01);
    assert(initial.collisions == std::vector<uint8_t>{1});
    const auto uploads = owned.statistics().scene_uploads;
    owned.setScene(Polygon(square), std::vector<Point>{{0, 0, .001}});
    const auto reused = owned.evaluate({{}}, .01);
    assert(reused.scene_version == initial.scene_version);
    assert(owned.statistics().scene_updates == 1 && owned.statistics().scene_reuses == 1);
    assert(owned.statistics().scene_uploads == uploads);
    Polygon mutable_shape = square;
    std::vector<Point> mutable_points{{2, 0, .001}};
    owned.setScene(mutable_shape, mutable_points);
    mutable_points[0].x = 0;
    mutable_shape[0].x = -3;
    assert(owned.evaluate({{}}, .01).collisions == std::vector<uint8_t>{0});
    owned.setScene(square, mutable_points);
    const auto changed = owned.evaluate({{}}, .01);
    assert(changed.collisions == std::vector<uint8_t>{1});
    assert(changed.scene_version == initial.scene_version + 2);
    mutable_points[0].radius = .03;
    owned.setScene(square, mutable_points);
    assert(owned.evaluate({{}}, .01).scene_version == changed.scene_version + 1);
    const Polygon larger{{-.4, -.4}, {.4, -.4}, {.4, .4}, {-.4, .4}};
    owned.setScene(larger, {{.3, 0, 0}});
    assert(owned.evaluate({{}}, 0).collisions == std::vector<uint8_t>{1});
    const auto before_invalid = owned.evaluate({{}}, 0).scene_version;
    rejects([&] { owned.setScene({}, {}); });
    assert(owned.evaluate({{}}, 0).scene_version == before_invalid);
    if (use_gpu && owned.available())
      assert(owned.statistics().scene_uploads == owned.statistics().scene_updates);
  }
  const std::vector<Point> point{{0, 0, 0}};
  automatic.setScene(square, point);
  rejects([&] { automatic.evaluate({{0, 0, std::numeric_limits<double>::quiet_NaN()}}, .010); });
  rejects([&] { automatic.evaluate({{}}, -.1); });
  rejects([&] { automatic.evaluate({}, -.1); });
  rejects([&] { automatic.evaluate({{}}, std::vector<double>{}); });
  rejects([&] { automatic.evaluate({{}}, std::vector<double>{-.1}); });
  rejects([&] { automatic.evaluate({{}}, {.01}, {}); });
  rejects([&] { automatic.evaluate({{}}, {.01}, {{1, 0, 0, 1}}); });
  rejects([&] { automatic.clearance({{}}, -.1); });
  rejects([&] { automatic.clearance({{}}, std::numeric_limits<double>::infinity()); });
  rejects([&] { automatic.setScene({}, {}); });
  rejects([&] { automatic.setScene({{-.2, -.2}, {-.2, .2}, {.2, .2}, {.2, -.2}}, {}); });
  rejects([&] { automatic.setScene({{0, 0}, {1, 0}, {2, 0}}, {}); });
  rejects([&] { automatic.setScene(square, {{0, 0, -.1}}); });
  GeometryCompute sparse({true, std::numeric_limits<uint64_t>::max()});
  sparse.setScene(square, point);
  assert(!sparse.willUseCuda(1));
  assert(!sparse.evaluate({{}}, .010).used_cuda);
  assert(sparse.statistics().cuda_batches == 0);
  GeometryCompute adaptive({true, 1000});
  std::vector<Point> far_points;
  for (int i = 0; i < 4000; ++i)
    far_points.push_back({10 + i * .001, 10, 0});
  adaptive.setScene(square, far_points);
  if (adaptive.available())
    assert(adaptive.willUseCuda(1));
  assert(!adaptive.evaluate({{}}, .01).used_cuda);
  assert(adaptive.statistics().last_candidate_work == 0);
  if (adaptive.available())
    assert(adaptive.statistics().candidate_cpu_dispatches == 1);
  GeometryCompute forced({true, 0});
  forced.setScene(square, far_points);
  assert(forced.evaluate({{}}, .01).used_cuda == forced.available());
  const auto &stats = automatic.statistics();
  assert(cpu.statistics().indexed_batches > 0);
  assert(cpu.statistics().full_scan_batches > 0);
  if (required)
    assert(stats.boundary_rechecks > 0);
  std::cout << "GEOMETRY_CHECKS comparisons=" << comparisons << " device=" << automatic.deviceName()
            << " clearance_comparisons=" << clearance_comparisons
            << " variable_margin_comparisons=" << variable_margin_comparisons
            << " bounded_comparisons=" << bounded_comparisons
            << " cuda_batches=" << stats.cuda_batches
            << " boundary_rechecks=" << stats.boundary_rechecks
            << " scene_updates=" << stats.scene_updates << " scene_reuses=" << stats.scene_reuses
            << " scene_uploads=" << stats.scene_uploads
            << " indexed_batches=" << stats.indexed_batches << " passed\n";
}
