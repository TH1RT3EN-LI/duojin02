#include "duojin02_navigation/gpu_geometry.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <random>

using namespace duojin02_navigation;
template<class F> void rejects(F &&f) {
  bool failed = false;
  try { f(); } catch (const std::exception &) { failed = true; }
  assert(failed);
}
int main(int argc, char **argv) {
  const std::string mode = argc > 1 ? argv[1] : "";
  const bool required = mode == "--require-cuda";
  GeometryCompute cpu({false, 0});
  GeometryCompute automatic({true, 0});
  assert(!cpu.available());
  if (mode == "--no-device") assert(!automatic.available());
  if (required && !automatic.available()) {
    std::cout << "CUDA device unavailable: " << automatic.deviceName() << '\n';
    return 77;
  }
  const Polygon polygon{{.213,.191465},{.201465,.203},{-.202879,.203},{-.214,.191879},
    {-.214,-.190465},{-.201465,-.203},{.202879,-.203},{.213,-.192879}};
  std::mt19937_64 random(20261010);
  std::uniform_real_distribution<double> xy(-3., 3.), angle(-3.14159, 3.14159);
  size_t comparisons = 0;
  auto compare = [&](const Polygon &shape, const std::vector<Point> &points, const std::vector<Pose> &poses, double margin) {
    cpu.setScene(shape, points);
    automatic.setScene(shape, points);
    const auto a = cpu.evaluate(poses, margin), b = automatic.evaluate(poses, margin);
    assert(a.collisions == b.collisions);
    if (required && !points.empty() && !poses.empty()) assert(b.used_cuda);
    for (size_t i = 0; i < poses.size(); ++i) {
      assert(static_cast<bool>(a.collisions[i]) == collision(poses[i], shape, points, margin));
      ++comparisons;
    }
  };
  for (int scene = 0; scene < 24; ++scene) {
    std::vector<Point> points;
    std::vector<Pose> poses;
    for (int i = 0; i < 1000; ++i) points.push_back({xy(random), xy(random), .007071067812 + (i%7)*.0005});
    points.push_back({0,0,.001});
    points.push_back({.223,0,0});
    for (int i = 0; i < 192; ++i) poses.push_back({xy(random), xy(random), angle(random)});
    poses.push_back({0,0,0});
    poses.push_back({.5,0,0});
    poses.push_back({0,0,1.5707963267948966});
    compare(polygon, points, poses, .010);
    // Each new epoch must replace all old device points, including shrinkage.
    compare(polygon, {{2.9,2.9,0}}, poses, .010);
    compare(polygon, {}, poses, .010);
  }
  const Polygon square{{-.2,-.2},{.2,-.2},{.2,.2},{-.2,.2}};
  // Exact edge/corner contacts and adjacent representable doubles, including
  // the inside-polygon cross-product tolerance used by the CPU predicate.
  for (double radius : {0., .007071067812}) {
    const double boundary = .2 + .010 + radius;
    for (double x : {boundary, std::nextafter(boundary, 0.), std::nextafter(boundary, 1.),
                    boundary-1e-9, boundary+1e-9}) {
      compare(square, {{x,0,radius}}, {{0,0,0},{0,0,1.5707963267948966}}, .010);
      compare(square, {{x,x,radius}}, {{0,0,0}}, .010);
    }
  }
  for (double x : {.2, .2+2.5e-10, std::nextafter(.2+2.5e-10,0.), std::nextafter(.2+2.5e-10,1.)})
    compare(square, {{x,0,0}}, {{0,0,0}}, 0.);
  for (double yaw : {0., .1, .5235987755982988, .7, 1.5707963267948966, 2.4, -2.9})
    for (double origin : {0., 1e3, 1e6}) {
      const Pose query{origin, -origin, yaw};
      for (const auto &local : std::vector<Point>{{.21,0,0},{-.21,0,0},{0,.21,0},{0,-.21,0},
                                                {.20707106781186548,.20707106781186548,0},
                                                {-.20707106781186548,-.20707106781186548,0}}) {
        const auto world = transform(local, query);
        for (double x : {world.x, std::nextafter(world.x,-std::numeric_limits<double>::infinity()),
                         std::nextafter(world.x,std::numeric_limits<double>::infinity())})
          compare(square, {{x,world.y,0}}, {query}, .010);
      }
    }
  for (double origin : {1e3, 1e6, 1e9, 1e12})
    for (double x : {.21, std::nextafter(.21,0.), std::nextafter(.21,1.), .21-1e-5, .21+1e-5})
      compare(square, {{origin+x, origin,0}}, {{origin,origin,0}}, .010);
  compare(square, {{0,0,0}}, {}, .010);
  const std::vector<Point> point{{0,0,0}};
  automatic.setScene(square, point);
  rejects([&] { automatic.evaluate({{0,0,std::numeric_limits<double>::quiet_NaN()}}, .010); });
  rejects([&] { automatic.evaluate({{}}, -.1); });
  rejects([&] { automatic.setScene({}, {}); });
  rejects([&] { automatic.setScene({{-.2,-.2},{-.2,.2},{.2,.2},{.2,-.2}}, {}); });
  rejects([&] { automatic.setScene({{0,0},{1,0},{2,0}}, {}); });
  rejects([&] { automatic.setScene(square, {{0,0,-.1}}); });
  GeometryCompute sparse({true,std::numeric_limits<uint64_t>::max()});
  sparse.setScene(square, point);
  assert(!sparse.willUseCuda(1));
  assert(!sparse.evaluate({{}},.010).used_cuda);
  assert(sparse.statistics().cuda_batches == 0);
  const auto &stats = automatic.statistics();
  if (required) assert(stats.boundary_rechecks > 0);
  std::cout << "GEOMETRY_CHECKS comparisons=" << comparisons << " device=" << automatic.deviceName()
    << " cuda_batches=" << stats.cuda_batches << " boundary_rechecks=" << stats.boundary_rechecks << " passed\n";
}
