// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "sil_carla_bridge/frame_transform.hpp"

namespace sil_carla_bridge
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Pose IdentityPose()
{
  return Pose{};
}

TEST(FrameTransformTest, IdentityPoseOnlyFlipsTheYAxis)
{
  const FrameTransform transform(IdentityPose());

  // CARLA (x forward, y right) -> ROS (x forward, y left).
  const Eigen::Vector3d ego = transform.WorldToEgo(Eigen::Vector3d(10.0, 4.0, 2.0));
  EXPECT_NEAR(10.0, ego.x(), 1e-12);
  EXPECT_NEAR(-4.0, ego.y(), 1e-12);
  EXPECT_NEAR(2.0, ego.z(), 1e-12);
}

TEST(FrameTransformTest, IdentityPoseReflectsTheRotationMatrix)
{
  const FrameTransform transform(IdentityPose());
  const Eigen::Matrix3d expected = HandednessReflection();
  EXPECT_LT((transform.rotation() - expected).norm(), 1e-12);
}

TEST(FrameTransformTest, HandednessReflectionIsItsOwnInverse)
{
  const Eigen::Matrix3d f = HandednessReflection();
  EXPECT_LT((f * f - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_LT((f.transpose() - f).norm(), 1e-12);
}

TEST(FrameTransformTest, TranslationSubtractsTheEgoOrigin)
{
  Pose ego_pose;
  ego_pose.position = Eigen::Vector3d(100.0, 200.0, 5.0);
  const FrameTransform transform(ego_pose);

  const Eigen::Vector3d ego = transform.WorldToEgo(Eigen::Vector3d(110.0, 200.0, 5.0));
  EXPECT_NEAR(10.0, ego.x(), 1e-12);
  EXPECT_NEAR(0.0, ego.y(), 1e-12);
  EXPECT_NEAR(0.0, ego.z(), 1e-12);
}

TEST(FrameTransformTest, EgoOriginMapsToTheOrigin)
{
  Pose ego_pose;
  ego_pose.position = Eigen::Vector3d(50.0, -30.0, 1.0);
  ego_pose.rpy = Eigen::Vector3d(0.0, 0.0, 1.2);
  const FrameTransform transform(ego_pose);

  EXPECT_LT(transform.WorldToEgo(ego_pose.position).norm(), 1e-12);
}

TEST(FrameTransformTest, EgoToWorldInvertsWorldToEgo)
{
  Pose ego_pose;
  ego_pose.position = Eigen::Vector3d(12.0, -4.0, 1.5);
  ego_pose.rpy = Eigen::Vector3d(0.05, -0.03, 0.9);
  const FrameTransform transform(ego_pose);

  const Eigen::Vector3d world_point(30.0, 8.0, -1.0);
  const Eigen::Vector3d round_trip =
    transform.EgoToWorld(transform.WorldToEgo(world_point));

  EXPECT_LT((round_trip - world_point).norm(), 1e-9);
}

TEST(FrameTransformTest, RotationMatrixIsOrthogonal)
{
  Pose ego_pose;
  ego_pose.rpy = Eigen::Vector3d(0.3, -0.2, 1.1);
  const FrameTransform transform(ego_pose);

  const Eigen::Matrix3d r = transform.rotation();
  EXPECT_LT((r * r.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  // A reflection has determinant -1, which is what fixes the handedness.
  EXPECT_NEAR(-1.0, r.determinant(), 1e-12);
}

TEST(FrameTransformTest, EgoYawMatchesTheConfiguredHeading)
{
  Pose ego_pose;
  ego_pose.rpy = Eigen::Vector3d(0.0, 0.0, kPi / 2.0);
  const FrameTransform transform(ego_pose);

  EXPECT_NEAR(kPi / 2.0, transform.EgoYaw(), 1e-9);
}

TEST(FrameTransformTest, EgoYawIsZeroForAnAxisAlignedEgo)
{
  const FrameTransform transform(IdentityPose());
  EXPECT_NEAR(0.0, transform.EgoYaw(), 1e-12);
}

TEST(FrameTransformTest, TargetAheadOfAnAlignedEgoStaysOnTheAxis)
{
  const FrameTransform transform(IdentityPose());
  const Eigen::Vector3d ego = transform.WorldToEgo(Eigen::Vector3d(40.0, 0.0, 0.0));

  EXPECT_NEAR(40.0, ego.x(), 1e-12);
  EXPECT_NEAR(0.0, ego.y(), 1e-12);
}

TEST(FrameTransformTest, TargetDirectlyAheadOfARotatedEgoHasNoLateralOffset)
{
  // Ego heading 90 degrees, so world +y is "forward" in the ego frame.
  Pose ego_pose;
  ego_pose.rpy = Eigen::Vector3d(0.0, 0.0, kPi / 2.0);
  const FrameTransform transform(ego_pose);

  const Eigen::Vector3d ego = transform.WorldToEgo(Eigen::Vector3d(0.0, 20.0, 0.0));
  EXPECT_NEAR(20.0, ego.x(), 1e-9);
  EXPECT_NEAR(0.0, ego.y(), 1e-9);
}

TEST(FrameTransformTest, WorldYawToEgoYawMatchesTheTransformedDirection)
{
  Pose ego_pose;
  ego_pose.rpy = Eigen::Vector3d(0.0, 0.0, 0.7);
  const FrameTransform transform(ego_pose);

  const double world_yaw = 1.1;
  const double ego_yaw = transform.WorldYawToEgoYaw(world_yaw);

  const Eigen::Vector3d world_direction(
    std::cos(world_yaw), std::sin(world_yaw), 0.0);
  const Eigen::Vector3d ego_direction = transform.RotateVectorToEgo(world_direction);

  EXPECT_NEAR(std::atan2(ego_direction.y(), ego_direction.x()), ego_yaw, 1e-12);
}

TEST(FrameTransformTest, WorldYawToEgoYawFlipsTheSignUnderTheReflection)
{
  // With an identity ego pose, the reflection alone negates a yaw.
  const FrameTransform transform(IdentityPose());
  EXPECT_NEAR(-0.5, transform.WorldYawToEgoYaw(0.5), 1e-12);
}

TEST(FrameTransformTest, RotateVectorToEgoIgnoresTranslation)
{
  Pose ego_pose;
  ego_pose.position = Eigen::Vector3d(1000.0, 1000.0, 0.0);
  const FrameTransform transform(ego_pose);

  const Eigen::Vector3d ego_vector =
    transform.RotateVectorToEgo(Eigen::Vector3d(1.0, 2.0, 3.0));

  // Only the y component flips; a translation must not leak into a vector.
  EXPECT_NEAR(1.0, ego_vector.x(), 1e-12);
  EXPECT_NEAR(-2.0, ego_vector.y(), 1e-12);
  EXPECT_NEAR(3.0, ego_vector.z(), 1e-12);
}

TEST(FrameTransformTest, RotationFromRpyMatchesExplicitComposition)
{
  const Eigen::Vector3d rpy(0.2, -0.4, 1.3);
  const Eigen::Matrix3d computed = RotationFromRpy(rpy);

  const Eigen::Matrix3d expected =
    (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();

  EXPECT_LT((computed - expected).norm(), 1e-12);
}

TEST(FrameTransformTest, RotationFromRpyOfZeroIsIdentity)
{
  EXPECT_LT((RotationFromRpy(Eigen::Vector3d::Zero()) - Eigen::Matrix3d::Identity()).norm(),
    1e-12);
}

TEST(FrameTransformTest, DegreesToRadiansConvertsTheExpectedValues)
{
  EXPECT_NEAR(kPi, DegreesToRadians(180.0), 1e-12);
  EXPECT_NEAR(kPi / 2.0, DegreesToRadians(90.0), 1e-12);
  EXPECT_DOUBLE_EQ(0.0, DegreesToRadians(0.0));
  EXPECT_NEAR(-kPi / 4.0, DegreesToRadians(-45.0), 1e-12);
}

TEST(FrameTransformTest, DistanceBetweenTwoWorldPointsIsPreserved)
{
  // The transform is rigid, so it must not change distances.
  Pose ego_pose;
  ego_pose.position = Eigen::Vector3d(5.0, 5.0, 0.0);
  ego_pose.rpy = Eigen::Vector3d(0.1, 0.0, 0.6);
  const FrameTransform transform(ego_pose);

  const Eigen::Vector3d a(10.0, 20.0, 1.0);
  const Eigen::Vector3d b(14.0, 17.0, 2.5);
  const double world_distance = (a - b).norm();
  const double ego_distance = (transform.WorldToEgo(a) - transform.WorldToEgo(b)).norm();

  EXPECT_NEAR(world_distance, ego_distance, 1e-9);
}

TEST(FrameTransformTest, DefaultConstructedTransformIsIdentity)
{
  const FrameTransform transform;
  EXPECT_LT((transform.rotation() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_LT(transform.translation().norm(), 1e-12);
}

}  // namespace
}  // namespace sil_carla_bridge
