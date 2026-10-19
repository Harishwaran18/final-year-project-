// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "sil_sensor_fusion/camera_projection.hpp"

namespace sil_sensor_fusion
{
namespace
{

CameraIntrinsics MakeIntrinsics()
{
  CameraIntrinsics intrinsics;
  intrinsics.fx = 1000.0;
  intrinsics.fy = 1000.0;
  intrinsics.cx = 640.0;
  intrinsics.cy = 360.0;
  intrinsics.image_width = 1280;
  intrinsics.image_height = 720;
  return intrinsics;
}

TEST(CameraProjectionTest, IntrinsicsValidity)
{
  CameraIntrinsics intrinsics = MakeIntrinsics();
  EXPECT_TRUE(intrinsics.valid());

  intrinsics.fx = 0.0;
  EXPECT_FALSE(intrinsics.valid());

  intrinsics = MakeIntrinsics();
  intrinsics.fy = 0.0;
  EXPECT_FALSE(intrinsics.valid());
}

TEST(CameraProjectionTest, PrincipalPointMapsToZeroBearing)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  EXPECT_DOUBLE_EQ(0.0, PixelToBearing(intrinsics.cx, intrinsics));
}

TEST(CameraProjectionTest, BearingIsPositiveToTheLeft)
{
  // ROS REP-103 has y to the left. A pixel left of the principal point (smaller
  // u) corresponds to a negative bearing under the atan((u - cx) / fx) model,
  // and a pixel to the right gives a positive bearing. The fusion node and the
  // bridge must agree on this convention, so it is pinned here.
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  EXPECT_LT(PixelToBearing(intrinsics.cx - 100.0, intrinsics), 0.0);
  EXPECT_GT(PixelToBearing(intrinsics.cx + 100.0, intrinsics), 0.0);
}

TEST(CameraProjectionTest, BearingMatchesThePinholeFormula)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  const double u = 840.0;  // 200 px right of the principal point
  EXPECT_NEAR(std::atan(200.0 / 1000.0), PixelToBearing(u, intrinsics), 1e-12);
}

TEST(CameraProjectionTest, BearingIsZeroForInvalidIntrinsics)
{
  CameraIntrinsics intrinsics = MakeIntrinsics();
  intrinsics.fx = 0.0;
  EXPECT_DOUBLE_EQ(0.0, PixelToBearing(1000.0, intrinsics));
}

TEST(CameraProjectionTest, BearingIsMonotonicInPixelColumn)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  double previous = PixelToBearing(0.0, intrinsics);
  for (double u = 10.0; u < 1280.0; u += 10.0) {
    const double current = PixelToBearing(u, intrinsics);
    EXPECT_GT(current, previous) << "u " << u;
    previous = current;
  }
}

TEST(CameraProjectionTest, ClassHeightsCoverTheCarlaClasses)
{
  EXPECT_DOUBLE_EQ(1.5, ClassHeight("car"));
  EXPECT_DOUBLE_EQ(3.2, ClassHeight("truck"));
  EXPECT_DOUBLE_EQ(3.2, ClassHeight("bus"));
  EXPECT_DOUBLE_EQ(1.75, ClassHeight("pedestrian"));
  EXPECT_GT(ClassHeight("motorcycle"), 0.0);
  EXPECT_GT(ClassHeight("bicycle"), 0.0);
  EXPECT_DOUBLE_EQ(0.0, ClassHeight("unknown"));
  EXPECT_DOUBLE_EQ(0.0, ClassHeight(""));
}

TEST(CameraProjectionTest, RangeInverselyScalesWithApparentHeight)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();

  BoundingBox2D large;
  large.class_label = "car";
  large.height = 100;
  large.width = 200;

  BoundingBox2D small = large;
  small.height = 50;

  const double near_range = EstimateRangeFromHeight(large, intrinsics);
  const double far_range = EstimateRangeFromHeight(small, intrinsics);

  EXPECT_NEAR(2.0 * near_range, far_range, 1e-9);
}

TEST(CameraProjectionTest, RangeMatchesThePinholeFormula)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  BoundingBox2D box;
  box.class_label = "car";  // 1.5 m tall
  box.height = 100;
  box.width = 200;

  // Z = fy * H / h = 1000 * 1.5 / 100 = 15 m.
  EXPECT_NEAR(15.0, EstimateRangeFromHeight(box, intrinsics), 1e-9);
}

TEST(CameraProjectionTest, RangeIsZeroForUnknownClass)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  BoundingBox2D box;
  box.class_label = "spaceship";
  box.height = 100;

  EXPECT_DOUBLE_EQ(0.0, EstimateRangeFromHeight(box, intrinsics));
}

TEST(CameraProjectionTest, RangeIsZeroForAZeroHeightBox)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  BoundingBox2D box;
  box.class_label = "car";
  box.height = 0;

  EXPECT_DOUBLE_EQ(0.0, EstimateRangeFromHeight(box, intrinsics));
}

TEST(CameraProjectionTest, RangeIsClampedToTheConfiguredBounds)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();

  BoundingBox2D huge;
  huge.class_label = "car";
  huge.height = 5000;  // Implies a sub-metre range.
  EXPECT_GE(EstimateRangeFromHeight(huge, intrinsics), 2.0);

  BoundingBox2D tiny;
  tiny.class_label = "car";
  tiny.height = 1;  // Implies a 1500 m range.
  EXPECT_LE(EstimateRangeFromHeight(tiny, intrinsics), 120.0);
}

TEST(CameraProjectionTest, ObservationIsValidForAKnownClass)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  BoundingBox2D box;
  box.class_label = "car";
  box.x_offset = 500;
  box.y_offset = 300;
  box.width = 200;
  box.height = 100;

  const CameraObservation observation = MakeCameraObservation(box, intrinsics);

  EXPECT_TRUE(observation.range_valid);
  EXPECT_GT(observation.range, 0.0);
  EXPECT_GT(observation.range_sigma, 0.0);
  EXPECT_GT(observation.bearing_sigma, 0.0);
  EXPECT_EQ("car", observation.class_label);
}

TEST(CameraProjectionTest, ObservationIsInvalidForAnUnknownClass)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  BoundingBox2D box;
  box.class_label = "unknown";
  box.height = 100;

  const CameraObservation observation = MakeCameraObservation(box, intrinsics);

  EXPECT_FALSE(observation.range_valid);
  EXPECT_DOUBLE_EQ(0.0, observation.range);
  EXPECT_DOUBLE_EQ(0.0, observation.range_sigma);
}

TEST(CameraProjectionTest, BearingSigmaGrowsAtTheImageEdges)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();

  BoundingBox2D centre;
  centre.class_label = "car";
  centre.width = 100;
  centre.height = 100;
  centre.x_offset = 590;  // Centred on the principal point.
  centre.y_offset = 310;

  BoundingBox2D edge = centre;
  edge.x_offset = 1200;  // Hard against the right edge.

  const CameraObservation centre_observation = MakeCameraObservation(centre, intrinsics);
  const CameraObservation edge_observation = MakeCameraObservation(edge, intrinsics);

  EXPECT_GT(edge_observation.bearing_sigma, centre_observation.bearing_sigma);
}

TEST(CameraProjectionTest, RangeSigmaGrowsWithDistance)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();

  BoundingBox2D near_box;
  near_box.class_label = "car";
  near_box.height = 200;
  near_box.width = 400;

  BoundingBox2D far_box = near_box;
  far_box.height = 20;

  const CameraObservation near_observation = MakeCameraObservation(near_box, intrinsics);
  const CameraObservation far_observation = MakeCameraObservation(far_box, intrinsics);

  ASSERT_TRUE(near_observation.range_valid);
  ASSERT_TRUE(far_observation.range_valid);
  EXPECT_GT(far_observation.range_sigma, near_observation.range_sigma);
}

TEST(CameraProjectionTest, BoundingBoxCenterIsTheMidpoint)
{
  BoundingBox2D box;
  box.x_offset = 100;
  box.y_offset = 200;
  box.width = 50;
  box.height = 40;

  const Eigen::Vector2d center = box.Center();
  EXPECT_DOUBLE_EQ(125.0, center.x());
  EXPECT_DOUBLE_EQ(220.0, center.y());
}

TEST(CameraProjectionTest, ObservationWithInvalidIntrinsicsHasNoRange)
{
  CameraIntrinsics intrinsics = MakeIntrinsics();
  intrinsics.fx = 0.0;
  intrinsics.fy = 0.0;

  BoundingBox2D box;
  box.class_label = "car";
  box.height = 100;
  box.width = 200;

  const CameraObservation observation = MakeCameraObservation(box, intrinsics);
  EXPECT_FALSE(observation.range_valid);
  EXPECT_DOUBLE_EQ(0.0, observation.bearing);
}

TEST(CameraProjectionTest, RangeSigmaHasAFloorForVeryNearTargets)
{
  const CameraIntrinsics intrinsics = MakeIntrinsics();
  BoundingBox2D box;
  box.class_label = "car";
  box.height = 600;  // Implies a ~2.5 m range.
  box.width = 1200;

  const CameraObservation observation = MakeCameraObservation(box, intrinsics);
  ASSERT_TRUE(observation.range_valid);
  EXPECT_GE(observation.range_sigma, 0.5);
}

}  // namespace
}  // namespace sil_sensor_fusion
