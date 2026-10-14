// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "lslidar_driver/self_filter.hpp"

using lslidar_driver::SelfFilter;
using lslidar_driver::SelfFilterConfig;
using Classification = lslidar_driver::SelfFilterClassification;

namespace {
SelfFilterConfig rectangle() {
  // Synthetic solid prism for geometry tests; these are not hardware calibration.
  SelfFilterConfig c;
  c.enabled = true;
  c.laser_to_body_xyz_m = {0.0924721442817, -0.00000897794961929, 0.14330766582};
  c.laser_to_body_rpy_rad = {0.0, 0.0, 0.0};
  c.body_polygon_xy_m = {-0.214, -0.203, 0.213, -0.203, 0.213, 0.203, -0.214, 0.203};
  c.body_z_bounds_m = {0.12, 0.16};
  return c;
}

TEST(SelfFilterTest, OffsetNonCircularBodyPreservesExternalNearReturns) {
  const SelfFilter filter(rectangle());
  EXPECT_FALSE(filter.shouldRemove({0.15, 0.0, 0.0}));  // 15 cm ahead is outside the solid.
  EXPECT_FALSE(filter.shouldRemove({0.0, 0.25, 0.0}));  // 25 cm side wall is outside.
  EXPECT_TRUE(filter.shouldRemove({-0.30, 0.0, 0.0}));  // 30 cm behind still lies inside.
  EXPECT_TRUE(filter.shouldRemove({0.10, 0.0, 0.0}));
}

TEST(SelfFilterTest, ConcaveIndentationRemainsObservable) {
  auto config = rectangle();
  config.laser_to_body_xyz_m = {0.0, 0.0, 0.0};
  config.body_z_bounds_m = {-0.1, 0.1};
  config.body_polygon_xy_m = {0.0, 0.0, 0.3, 0.0, 0.3, 0.1, 0.1, 0.1, 0.1, 0.3, 0.0, 0.3};
  const SelfFilter filter(config);
  EXPECT_TRUE(filter.shouldRemove({0.05, 0.2, 0.0}));
  EXPECT_FALSE(filter.shouldRemove({0.2, 0.2, 0.0}));
  EXPECT_TRUE(filter.shouldRemove({0.2, 0.05, 0.0}));
  const auto vertices = config.body_polygon_xy_m;
  for (size_t i = 0; i < vertices.size(); i += 2) {
    config.body_polygon_xy_m[i] = vertices[vertices.size() - i - 2];
    config.body_polygon_xy_m[i + 1] = vertices[vertices.size() - i - 1];
  }
  const SelfFilter reversed(config);
  EXPECT_TRUE(reversed.shouldRemove({0.2, 0.05, 0.0}));
  EXPECT_FALSE(reversed.shouldRemove({0.2, 0.2, 0.0}));
}

TEST(SelfFilterTest, HeightAndFullRigidTransformDetermineTheMask) {
  auto config = rectangle();
  config.laser_to_body_xyz_m = {0.0, 0.0, 0.2};
  config.laser_to_body_rpy_rad = {0.0, std::acos(-1.0) / 2.0, std::acos(-1.0) / 2.0};
  config.body_z_bounds_m = {0.05, 0.15};
  const SelfFilter tilted(config);
  auto transformed = tilted.toBody({0.1, 0.05, 0.0});
  EXPECT_NEAR(transformed.x, -0.05, 1e-12);
  EXPECT_NEAR(transformed.y, 0.0, 1e-12);
  EXPECT_NEAR(transformed.z, 0.1, 1e-12);
  EXPECT_TRUE(tilted.shouldRemove({0.1, 0.05, 0.0}));
  EXPECT_FALSE(tilted.shouldRemove({0.02, 0.05, 0.0})); // XY same, outside prism height.
  config.laser_to_body_rpy_rad = {std::acos(-1.0) / 2.0, 0.0, 0.0};
  const SelfFilter rolled(config);
  transformed = rolled.toBody({0.1, -0.1, 0.05});
  EXPECT_NEAR(transformed.x, 0.1, 1e-12);
  EXPECT_NEAR(transformed.y, -0.05, 1e-12);
  EXPECT_NEAR(transformed.z, 0.1, 1e-12);
  EXPECT_TRUE(rolled.shouldRemove({0.1, -0.1, 0.05}));
}

TEST(SelfFilterTest, AmbiguityBandPreservesBothSidesAndAllPrismSurfaces) {
  auto config = rectangle();
  config.laser_to_body_xyz_m = {0.0, 0.0, 0.0};
  config.body_z_bounds_m = {-0.1, 0.1};
  config.boundary_ambiguity_m = 0.01;
  const SelfFilter filter(config);
  EXPECT_EQ(filter.classify({0.208, 0.0, 0.0}), Classification::Ambiguous);
  EXPECT_EQ(filter.classify({0.218, 0.0, 0.0}), Classification::Ambiguous);
  EXPECT_EQ(filter.classify({0.19, 0.0, 0.0}), Classification::Inside);
  EXPECT_EQ(filter.classify({0.23, 0.0, 0.0}), Classification::Outside);
  EXPECT_EQ(filter.classify({0.0, 0.0, 0.095}), Classification::Ambiguous);
  EXPECT_EQ(filter.classify({0.0, 0.0, 0.105}), Classification::Ambiguous);
  EXPECT_EQ(filter.classify({0.0, 0.0, -0.095}), Classification::Ambiguous);
  EXPECT_EQ(filter.classify({0.0, 0.0, -0.105}), Classification::Ambiguous);
  config.boundary_ambiguity_m = 0.0;
  const SelfFilter exact(config);
  EXPECT_FALSE(exact.shouldRemove({0.213, 0.0, 0.0}));
  EXPECT_FALSE(exact.shouldRemove({0.0, 0.0, 0.1}));
  EXPECT_FALSE(exact.shouldRemove({0.0, 0.0, std::numeric_limits<double>::quiet_NaN()}));
}

TEST(SelfFilterTest, RequiresExplicitFiniteTransformAndHeightWhenEnabled) {
  auto c = rectangle();
  c.laser_to_body_xyz_m.clear(); EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.laser_to_body_rpy_rad.clear(); EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.body_z_bounds_m.clear(); EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.body_z_bounds_m = {0.2, 0.1}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.body_z_bounds_m = {0.1, 0.1}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.laser_to_body_xyz_m[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.boundary_ambiguity_m = -0.01; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.boundary_ambiguity_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
}

TEST(SelfFilterTest, StrictInteriorCropsFormerBandAndPreservesAdjacentFloat32Exterior) {
  auto config = rectangle();
  config.laser_to_body_xyz_m = {0.0, 0.0, 0.0};
  // Exactly representable boundaries make float32 neighbours unambiguous.
  config.body_polygon_xy_m = {-0.25, -0.25, 0.25, -0.25, 0.25, 0.25, -0.25, 0.25};
  config.body_z_bounds_m = {0.125, 0.25};
  config.boundary_ambiguity_m = 0.0;
  const SelfFilter strict(config);
  EXPECT_TRUE(strict.shouldRemove({0.25 - 0.028, 0.0, 0.1875}));
  EXPECT_TRUE(strict.shouldRemove({std::nextafter(0.25f, 0.0f), 0.0, 0.1875}));
  EXPECT_FALSE(strict.shouldRemove({0.25, 0.0, 0.1875}));
  EXPECT_FALSE(strict.shouldRemove({std::nextafter(0.25f, 1.0f), 0.0, 0.1875}));
  EXPECT_TRUE(strict.shouldRemove({0.0, std::nextafter(-0.25f, 0.0f), 0.1875}));
  EXPECT_FALSE(strict.shouldRemove({0.0, std::nextafter(-0.25f, -1.0f), 0.1875}));
  EXPECT_TRUE(strict.shouldRemove({0.0, 0.0, std::nextafter(0.125f, 1.0f)}));
  EXPECT_FALSE(strict.shouldRemove({0.0, 0.0, 0.125}));
  EXPECT_FALSE(strict.shouldRemove({0.0, 0.0, std::nextafter(0.125f, 0.0f)}));
  EXPECT_TRUE(strict.shouldRemove({0.0, 0.0, std::nextafter(0.25f, 0.0f)}));
  EXPECT_FALSE(strict.shouldRemove({0.0, 0.0, 0.25}));
  config.body_polygon_xy_m = {-0.25, -0.125, -0.125, -0.25, 0.125, -0.25, 0.25, -0.125,
                             0.25, 0.125, 0.125, 0.25, -0.125, 0.25, -0.25, 0.125};
  const SelfFilter chamfer(config);
  EXPECT_FALSE(chamfer.shouldRemove({0.25, 0.125, 0.1875}));
  EXPECT_FALSE(chamfer.shouldRemove({0.1875, 0.1875, 0.1875}));
  EXPECT_TRUE(chamfer.shouldRemove({std::nextafter(0.1875f, 0.0f), 0.1875, 0.1875}));
  EXPECT_FALSE(chamfer.shouldRemove({std::nextafter(0.1875f, 1.0f), 0.1875, 0.1875}));
}

TEST(SelfFilterTest, RejectsSelfIntersectionsDegeneracyAndRepeatedVertices) {
  auto c = rectangle();
  c.body_polygon_xy_m = {0, 0, 1, 1, 0, 1, 1, 0}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c.body_polygon_xy_m = {0, 0, 1, 0, 2, 0}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c.body_polygon_xy_m = {0, 0, 1, 0, 1, 0, 0, 1}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c.body_polygon_xy_m = {0, 0, 1, 0, 0.5, 0, 1, 1, 0, 1}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c.body_polygon_xy_m = {0, 0, 1, 0, 1, 1, 0, 0, 0, 1}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c.body_polygon_xy_m = {0, 0, 1, 0, 1, 1, 0}; EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
  c = rectangle(); c.body_polygon_xy_m[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(SelfFilter{c}, std::invalid_argument);
}

TEST(SelfFilterTest, DisabledFilterDoesNotNeedGeometryAndNeverRemovesPoints) {
  const SelfFilter filter;
  EXPECT_FALSE(filter.enabled());
  EXPECT_FALSE(filter.shouldRemove({0.0, 0.0, 0.0}));
  EXPECT_FALSE(filter.shouldRemove({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}));
}
}  // namespace
