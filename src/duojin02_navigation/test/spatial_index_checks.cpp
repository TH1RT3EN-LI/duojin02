#include "duojin02_navigation/guard_query.hpp"
#include "spatial_index_reference.hpp"
#include <cassert>
#include <iostream>
#include <random>
#include <tuple>
using namespace duojin02_navigation;

int main() {
  std::mt19937_64 rng(20261010);
  std::uniform_real_distribution<double> unit(-1, 1);
  SpatialPointIndex candidate(.05);
  ReferenceSpatialPointIndex reference(.05);
  size_t compared = 0, dense_builds = 0, sparse_builds = 0;
  auto check = [&](const std::vector<Point> &points, double x, double y, double radius) {
    std::vector<size_t> a, b, full;
    const auto old = reference.forEachNearIndexed(points, x, y, radius,
                                                  [&](const Point &, size_t i) { a.push_back(i); });
    const auto now =
        candidate.forEachNearIndexed(points, x, y, radius, [&](const Point &p, size_t i) {
          assert(p.x == points[i].x && p.y == points[i].y && p.radius == points[i].radius);
          b.push_back(i);
        });
    assert(old == now && a == b);
    const double expanded = radius + 1e-8 + 1e-12 * (std::abs(x) + std::abs(y));
    auto bin = [](double value) { return static_cast<int>(std::floor(value / .05)); };
    const int lx = bin(x - expanded), ux = bin(x + expanded), ly = bin(y - expanded),
              uy = bin(y + expanded);
    size_t examined = 0;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto &p = points[i];
      const int bx = bin(p.x), by = bin(p.y);
      if (bx < lx || bx > ux || by < ly || by > uy)
        continue;
      ++examined;
      if (std::hypot(p.x - x, p.y - y) <= expanded)
        full.push_back(i);
    }
    std::sort(full.begin(), full.end(), [&](size_t i, size_t j) {
      return std::make_tuple(bin(points[i].x), bin(points[i].y), i) <
             std::make_tuple(bin(points[j].x), bin(points[j].y), j);
    });
    assert(examined == now && full == b);
    Point query{x, y, 0};
    const bool matched = reference.hasWithin(points, query, radius);
    assert(candidate.hasWithin(points, query, radius) == matched);
    bool expected = false;
    for (const auto &p : points)
      if (bin(p.x) >= bin(x - radius) && bin(p.x) <= bin(x + radius) &&
          bin(p.y) >= bin(y - radius) && bin(p.y) <= bin(y + radius) &&
          std::hypot(p.x - x, p.y - y) <= radius)
        expected = true;
    assert(matched == expected);
    ++compared;
  };
  for (int family = 0; family < 72; ++family) {
    std::vector<Point> points;
    const int count = family % 9 == 0 ? 0 : 64 + family * 11;
    const double scale = family % 2 ? 1.1 : 40.;
    const double origin = family % 7 == 0 ? 1e6 : -.2;
    for (int i = 0; i < count; ++i) {
      points.push_back(
          {origin + scale * unit(rng), -origin + scale * unit(rng), .005 + (i % 7) * .001});
      if (i > 0 && i % 19 == 0)
        points.back() = points.front();
    }
    candidate.rebuild(points);
    reference.rebuild(points);
    candidate.denseStorageCells() ? ++dense_builds : ++sparse_builds;
    assert(candidate.denseStorageCells() <= 65536 &&
           candidate.maxRadius() == reference.maxRadius());
    for (int query = 0; query < 80; ++query)
      check(points, origin + scale * unit(rng), -origin + scale * unit(rng),
            .02 + (query % 9) * .05);

    check(points, origin + scale + 2, -origin + scale + 2, .01);
    check(points, origin - scale - 2, -origin - scale - 2, .01);
  }
  std::vector<Point> boundary;
  for (double x : {-.1, std::nextafter(-.05, -1.), -.05, std::nextafter(-.05, 0.), -0., 0., .05,
                   std::nextafter(.05, 1.)})
    for (double y : {-.05, 0., .05})
      boundary.push_back({x, y, .01});
  candidate.rebuild(boundary);
  reference.rebuild(boundary);
  assert(candidate.denseStorageCells() > 0);
  for (const auto &p : boundary)
    for (double radius : {0., .05, std::nextafter(.05, 0.), std::nextafter(.05, 1.), .11})
      check(boundary, p.x, p.y, radius);

  for (int height : {256, 257}) {
    std::vector<Point> points;
    for (int x = 0; x < 256; x += 4)
      for (int y = 0; y < 256; y += 4)
        points.push_back({(x + .25) * .05, (y + .25) * .05, .01});
    points.push_back({255.25 * .05, (height - .75) * .05, .02});
    candidate.rebuild(points);
    reference.rebuild(points);
    assert(candidate.denseStorageCells() == (height == 256 ? 65536U : 0U));
    for (int i = 0; i < 8; ++i)
      check(points, .5 + i, 1.1 + i, .07);
  }
  for (int extent : {31, 32}) {
    std::vector<Point> points(64, {.025, .025, .01});
    points.back() = {(extent + .25) * .05, (extent + .25) * .05, .02};
    candidate.rebuild(points);
    reference.rebuild(points);
    assert(candidate.denseStorageCells() == (extent == 31 ? 1024U : 0U));
    check(points, .025, .025, .02);
  }

  std::vector<Point> hidden(64, {.025, .025, .01});
  hidden[17] = {40., 40., .08};
  candidate.rebuild(hidden);
  reference.rebuild(hidden);
  assert(candidate.denseStorageCells() == 0);
  check(hidden, .025, .025, .02);
  assert(candidate.maxRadius() == .08);
  candidate.rebuild({});
  reference.rebuild({});
  check({}, 0, 0, .1);
  assert(candidate.denseStorageCells() == 0 && candidate.maxRadius() == 0);
  assert(dense_builds > 0 && sparse_builds > 0);
  std::cout
      << "SPATIAL_INDEX_EQUIVALENCE queries=" << compared << " dense_builds=" << dense_builds
      << " sparse_or_empty_builds=" << sparse_builds
      << " ordered indices/point values, examined counts, hasWithin, full-vector reference, replacement and storage boundaries passed\n";
}
