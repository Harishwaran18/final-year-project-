// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "sil_sensor_fusion/cluster_geometry.hpp"

namespace sil_sensor_fusion
{
namespace
{

TEST(ClusterGeometryTest, RegionAcceptsPointsInsideTheBox)
{
  const ClusterConfig config;
  EXPECT_TRUE(IsPointInRegion(Eigen::Vector3d(10.0, 0.0, 0.0), config));
  EXPECT_TRUE(IsPointInRegion(Eigen::Vector3d(0.0, 20.0, 0.0), config));
  EXPECT_TRUE(IsPointInRegion(Eigen::Vector3d(30.0, 30.0, 1.0), config));
}

TEST(ClusterGeometryTest, RegionRejectsPointsTooCloseOrTooFar)
{
  const ClusterConfig config;
  EXPECT_FALSE(IsPointInRegion(Eigen::Vector3d(0.5, 0.0, 0.0), config));
  EXPECT_FALSE(IsPointInRegion(Eigen::Vector3d(100.0, 0.0, 0.0), config));
}

TEST(ClusterGeometryTest, RegionRejectsPointsOutsideTheHeightBand)
{
  const ClusterConfig config;
  EXPECT_FALSE(IsPointInRegion(Eigen::Vector3d(10.0, 0.0, -5.0), config));
  EXPECT_FALSE(IsPointInRegion(Eigen::Vector3d(10.0, 0.0, 10.0), config));
}

TEST(ClusterGeometryTest, RegionRejectsNonFinitePoints)
{
  const ClusterConfig config;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(IsPointInRegion(Eigen::Vector3d(nan, 0.0, 0.0), config));
  EXPECT_FALSE(IsPointInRegion(Eigen::Vector3d(
    0.0, std::numeric_limits<double>::infinity(), 0.0), config));
}

TEST(ClusterGeometryTest, RegionBoundariesAreInclusive)
{
  ClusterConfig config;
  config.min_range = 2.0;
  config.max_range = 50.0;
  config.min_height = -1.0;
  config.max_height = 2.0;

  EXPECT_TRUE(IsPointInRegion(Eigen::Vector3d(2.0, 0.0, 0.0), config));
  EXPECT_TRUE(IsPointInRegion(Eigen::Vector3d(50.0, 0.0, 0.0), config));
  EXPECT_TRUE(IsPointInRegion(Eigen::Vector3d(10.0, 0.0, 2.0), config));
  EXPECT_TRUE(IsPointInRegion(Eigen::Vector3d(10.0, 0.0, -1.0), config));
}

TEST(ClusterGeometryTest, CentroidOfEmptyInputIsTheOrigin)
{
  EXPECT_EQ(Eigen::Vector3d::Zero(), ComputeCentroid({}));
}

TEST(ClusterGeometryTest, CentroidIsTheArithmeticMean)
{
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 4.0, 6.0),
  };
  EXPECT_EQ(Eigen::Vector3d(1.0, 2.0, 3.0), ComputeCentroid(points));
}

TEST(ClusterGeometryTest, ExtentsOfEmptyInputAreZero)
{
  EXPECT_EQ(Eigen::Vector3d::Zero(), ComputeExtents({}));
}

TEST(ClusterGeometryTest, ExtentsAreTheAxisAlignedSpan)
{
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(-1.0, -2.0, -3.0),
    Eigen::Vector3d(1.0, 2.0, 3.0),
  };
  EXPECT_EQ(Eigen::Vector3d(2.0, 4.0, 6.0), ComputeExtents(points));
}

TEST(ClusterGeometryTest, SinglePointHasZeroExtent)
{
  const std::vector<Eigen::Vector3d> points{Eigen::Vector3d(1.0, 2.0, 3.0)};
  EXPECT_EQ(Eigen::Vector3d::Zero(), ComputeExtents(points));
}

TEST(ClusterGeometryTest, ClusterAcceptanceEnforcesPointCountBounds)
{
  const ClusterConfig config;  // min 8, max 4000
  const Eigen::Vector3d vehicle_extents(4.5, 1.8, 1.5);

  EXPECT_FALSE(IsClusterAccepted(0, vehicle_extents, config));
  EXPECT_FALSE(IsClusterAccepted(7, vehicle_extents, config));
  EXPECT_TRUE(IsClusterAccepted(8, vehicle_extents, config));
  EXPECT_TRUE(IsClusterAccepted(4000, vehicle_extents, config));
  EXPECT_FALSE(IsClusterAccepted(4001, vehicle_extents, config));
}

TEST(ClusterGeometryTest, ClusterAcceptanceRejectsThinClusters)
{
  const ClusterConfig config;
  // A single scan line has no lateral extent and must be treated as noise.
  const Eigen::Vector3d thin(0.01, 0.01, 0.0);
  EXPECT_FALSE(IsClusterAccepted(100, thin, config));

  const Eigen::Vector3d wide_enough(0.3, 0.3, 0.0);
  EXPECT_TRUE(IsClusterAccepted(100, wide_enough, config));
}

TEST(ClusterGeometryTest, RansacRecoversAPlaneAtTheOrigin)
{
  // A grid on z = 0, plus a handful of outliers well above it.
  std::vector<Eigen::Vector3d> points;
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      points.emplace_back(static_cast<double>(i) * 0.5,
        static_cast<double>(j) * 0.5, 0.0);
    }
  }
  for (int i = 0; i < 20; ++i) {
    points.emplace_back(static_cast<double>(i), 0.0, 5.0);
  }

  const ClusterConfig config;
  const PlaneModel plane = FitGroundPlaneRansac(points, config);

  ASSERT_TRUE(plane.valid);
  // |c| must be close to 1 for a horizontal plane, and |d| close to 0.
  EXPECT_NEAR(1.0, std::abs(plane.c), 0.05);
  EXPECT_NEAR(0.0, plane.d, 0.1);
  EXPECT_GT(plane.inlier_count, 400U);
}

TEST(ClusterGeometryTest, RansacRecoversAnOffsetPlane)
{
  std::vector<Eigen::Vector3d> points;
  const double ground_z = -1.7;
  for (int i = -10; i <= 10; ++i) {
    for (int j = -10; j <= 10; ++j) {
      points.emplace_back(static_cast<double>(i) * 0.5,
        static_cast<double>(j) * 0.5, ground_z);
    }
  }

  const ClusterConfig config;
  const PlaneModel plane = FitGroundPlaneRansac(points, config);

  ASSERT_TRUE(plane.valid);
  EXPECT_NEAR(1.0, std::abs(plane.c), 0.05);
  // With a unit normal along z, d == -ground_z.
  EXPECT_NEAR(-ground_z, std::abs(plane.d), 0.1);
}

TEST(ClusterGeometryTest, RansacIsDeterministic)
{
  std::vector<Eigen::Vector3d> points;
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> xy(-10.0, 10.0);
  std::uniform_real_distribution<double> z(0.0, 2.0);
  for (int i = 0; i < 300; ++i) {
    points.emplace_back(xy(rng), xy(rng), i % 3 == 0 ? 0.0 : z(rng));
  }

  const ClusterConfig config;
  const PlaneModel first = FitGroundPlaneRansac(points, config);
  const PlaneModel second = FitGroundPlaneRansac(points, config);

  EXPECT_EQ(first.a, second.a);
  EXPECT_EQ(first.b, second.b);
  EXPECT_EQ(first.c, second.c);
  EXPECT_EQ(first.d, second.d);
  EXPECT_EQ(first.inlier_count, second.inlier_count);
}

TEST(ClusterGeometryTest, RansacRejectsTooFewPoints)
{
  const ClusterConfig config;
  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
  };
  EXPECT_FALSE(FitGroundPlaneRansac(points, config).valid);
}

TEST(ClusterGeometryTest, RansacRejectsVerticalPlanes)
{
  // A wall of points in the x-z plane is not the ground.
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 30; ++i) {
    for (int j = 0; j < 30; ++j) {
      points.emplace_back(0.0, static_cast<double>(i) * 0.2,
        static_cast<double>(j) * 0.2);
    }
  }

  const ClusterConfig config;
  const PlaneModel plane = FitGroundPlaneRansac(points, config);
  // Either no plane is found, or the one found is horizontal.
  if (plane.valid) {
    EXPECT_GT(std::abs(plane.c), 0.85);
  }
}

TEST(ClusterGeometryTest, PlaneSignedDistanceHasTheExpectedSign)
{
  PlaneModel plane;
  plane.a = 0.0;
  plane.b = 0.0;
  plane.c = 1.0;
  plane.d = 0.0;

  EXPECT_DOUBLE_EQ(2.5, plane.SignedDistance(Eigen::Vector3d(1.0, 1.0, 2.5)));
  EXPECT_DOUBLE_EQ(-2.5, plane.SignedDistance(Eigen::Vector3d(1.0, 1.0, -2.5)));
  EXPECT_DOUBLE_EQ(0.0, plane.SignedDistance(Eigen::Vector3d(1.0, 1.0, 0.0)));
}

TEST(ClusterGeometryTest, ClusterRangeAndBearingUseTheCentroid)
{
  Cluster cluster;
  cluster.centroid = Eigen::Vector3d(3.0, 4.0, 0.0);

  EXPECT_DOUBLE_EQ(5.0, cluster.Range());
  EXPECT_NEAR(std::atan2(4.0, 3.0), cluster.Bearing(), 1e-12);
}

}  // namespace
}  // namespace sil_sensor_fusion
