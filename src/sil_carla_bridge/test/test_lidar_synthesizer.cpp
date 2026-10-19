// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "sil_carla_bridge/frame_transform.hpp"
#include "sil_carla_bridge/lidar_synthesizer.hpp"

namespace sil_carla_bridge
{
namespace
{

LidarSynthesizerConfig SmallConfig()
{
  LidarSynthesizerConfig config;
  config.channels = 8;
  config.azimuth_samples = 128;
  config.vertical_fov = 0.5;
  config.min_range = 1.0;
  config.max_range = 80.0;
  config.range_noise_std = 0.0;  // Deterministic for assertions.
  return config;
}

LidarSynthesizer::Box MakeBox(double x, double y, double half_x, double half_y,
  double half_z, double yaw = 0.0)
{
  LidarSynthesizer::Box box;
  box.center = Eigen::Vector3d(x, y, 0.0);
  box.half_extents = Eigen::Vector3d(half_x, half_y, half_z);
  box.yaw = yaw;
  box.intensity = 0.8;
  box.actor_id = 1;
  box.class_label = "car";
  return box;
}

TEST(LidarSynthesizerTest, RayDirectionIsUnitLength)
{
  const LidarSynthesizer synthesizer(SmallConfig());
  for (std::uint32_t channel = 0; channel < 8; ++channel) {
    for (std::uint32_t azimuth = 0; azimuth < 128; azimuth += 7) {
      const Eigen::Vector3d direction = synthesizer.RayDirection(channel, azimuth);
      EXPECT_NEAR(1.0, direction.norm(), 1e-12)
        << "channel " << channel << " azimuth " << azimuth;
    }
  }
}

TEST(LidarSynthesizerTest, RayDirectionSpansTheFullAzimuth)
{
  // A zero vertical FOV keeps the rays exactly horizontal, which makes the
  // azimuth assertions exact rather than trigonometric.
  LidarSynthesizerConfig config = SmallConfig();
  config.vertical_fov = 0.0;
  const LidarSynthesizer synthesizer(config);

  const Eigen::Vector3d first = synthesizer.RayDirection(0, 0);
  EXPECT_NEAR(1.0, first.x(), 1e-9);
  EXPECT_NEAR(0.0, first.y(), 1e-9);
  EXPECT_NEAR(0.0, first.z(), 1e-9);

  // A quarter of the way round the sweep is +90 degrees in azimuth.
  const Eigen::Vector3d quarter = synthesizer.RayDirection(0, 32);
  EXPECT_NEAR(0.0, quarter.x(), 1e-9);
  EXPECT_NEAR(1.0, quarter.y(), 1e-9);

  // Half way round is the opposite direction.
  const Eigen::Vector3d opposite = synthesizer.RayDirection(0, 64);
  EXPECT_NEAR(-1.0, opposite.x(), 1e-9);
  EXPECT_NEAR(0.0, opposite.y(), 1e-9);
}

TEST(LidarSynthesizerTest, RayDirectionElevationSweepsTopToBottom)
{
  const LidarSynthesizer synthesizer(SmallConfig());

  const Eigen::Vector3d top = synthesizer.RayDirection(0, 0);
  const Eigen::Vector3d bottom = synthesizer.RayDirection(7, 0);

  EXPECT_GT(top.z(), 0.0) << "the first channel must look upwards";
  EXPECT_LT(bottom.z(), 0.0) << "the last channel must look downwards";
}

TEST(LidarSynthesizerTest, RayBoxIntersectionHitsTheNearestFace)
{
  const LidarSynthesizer::Box box = MakeBox(10.0, 0.0, 1.0, 1.0, 1.0);
  double distance = 0.0;

  ASSERT_TRUE(LidarSynthesizer::IntersectRayBox(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), box, distance));
  // The near face is at x = 9.
  EXPECT_NEAR(9.0, distance, 1e-9);
}

TEST(LidarSynthesizerTest, RayBoxIntersectionMissesLaterally)
{
  const LidarSynthesizer::Box box = MakeBox(10.0, 0.0, 1.0, 1.0, 1.0);
  double distance = 0.0;

  EXPECT_FALSE(LidarSynthesizer::IntersectRayBox(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitY(), box, distance));
}

TEST(LidarSynthesizerTest, RayBoxIntersectionHandlesYaw)
{
  // A 45 degree box is wider across the x-axis than a 1 m half extent.
  const LidarSynthesizer::Box box = MakeBox(10.0, 0.0, 1.0, 1.0, 1.0, M_PI / 4.0);
  double distance = 0.0;

  ASSERT_TRUE(LidarSynthesizer::IntersectRayBox(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), box, distance));
  EXPECT_LT(distance, 9.0) << "the rotated corner must be closer";
  EXPECT_NEAR(10.0 - std::sqrt(2.0), distance, 1e-9);
}

TEST(LidarSynthesizerTest, RayBoxIntersectionRejectsABoxBehindTheOrigin)
{
  const LidarSynthesizer::Box box = MakeBox(-10.0, 0.0, 1.0, 1.0, 1.0);
  double distance = 0.0;

  EXPECT_FALSE(LidarSynthesizer::IntersectRayBox(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), box, distance));
}

TEST(LidarSynthesizerTest, RayBoxIntersectionHandlesARayParallelToASlab)
{
  // The ray travels along x while the box is offset in z: a parallel miss.
  LidarSynthesizer::Box box = MakeBox(10.0, 0.0, 1.0, 1.0, 1.0);
  box.center.z() = 5.0;
  double distance = 0.0;

  EXPECT_FALSE(LidarSynthesizer::IntersectRayBox(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), box, distance));
}

TEST(LidarSynthesizerTest, SynthesizeReturnsPointsOnTheBox)
{
  const LidarSynthesizer synthesizer(SmallConfig());
  const std::vector<LidarSynthesizer::Box> boxes{MakeBox(15.0, 0.0, 1.0, 1.0, 1.0)};

  const pcl::PointCloud<pcl::PointXYZI> cloud = synthesizer.Synthesize(boxes);

  ASSERT_FALSE(cloud.empty());
  EXPECT_EQ(cloud.size(), cloud.width);
  EXPECT_EQ(1U, cloud.height);

  for (const auto & point : cloud.points) {
    const double range = std::hypot(
      static_cast<double>(point.x), static_cast<double>(point.y));
    EXPECT_GT(range, 13.0);
    EXPECT_LT(range, 17.0);
  }
}

TEST(LidarSynthesizerTest, SynthesizeIsDeterministicWithoutNoise)
{
  const LidarSynthesizer synthesizer(SmallConfig());
  const std::vector<LidarSynthesizer::Box> boxes{
    MakeBox(12.0, 1.0, 1.0, 1.0, 1.0),
    MakeBox(30.0, -2.0, 1.0, 1.0, 1.0),
  };

  const pcl::PointCloud<pcl::PointXYZI> first = synthesizer.Synthesize(boxes);
  const pcl::PointCloud<pcl::PointXYZI> second = synthesizer.Synthesize(boxes);

  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_FLOAT_EQ(first.points[i].x, second.points[i].x);
    EXPECT_FLOAT_EQ(first.points[i].y, second.points[i].y);
    EXPECT_FLOAT_EQ(first.points[i].z, second.points[i].z);
  }
}

TEST(LidarSynthesizerTest, SynthesizeWithNoBoxesIsEmpty)
{
  const LidarSynthesizer synthesizer(SmallConfig());
  EXPECT_TRUE(synthesizer.Synthesize({}).empty());
}

TEST(LidarSynthesizerTest, MoreBoxesProduceMorePoints)
{
  // The boxes are placed side by side rather than one behind the other: with a
  // full 360 degree sweep a distant box is simply occluded by a nearer one, so
  // stacking them along the ego axis would not change the point count.
  const LidarSynthesizer synthesizer(SmallConfig());

  const pcl::PointCloud<pcl::PointXYZI> one = synthesizer.Synthesize(
    {MakeBox(15.0, 0.0, 1.0, 1.0, 1.0)});
  const pcl::PointCloud<pcl::PointXYZI> two = synthesizer.Synthesize(
    {MakeBox(15.0, 0.0, 1.0, 1.0, 1.0), MakeBox(0.0, 15.0, 1.0, 1.0, 1.0)});

  EXPECT_GT(two.size(), one.size());
}

TEST(LidarSynthesizerTest, IntensityIsCarriedThroughFromTheBox)
{
  const LidarSynthesizer synthesizer(SmallConfig());
  LidarSynthesizer::Box box = MakeBox(15.0, 0.0, 1.0, 1.0, 1.0);
  box.intensity = 0.42;

  const pcl::PointCloud<pcl::PointXYZI> cloud = synthesizer.Synthesize({box});
  ASSERT_FALSE(cloud.empty());
  for (const auto & point : cloud.points) {
    EXPECT_FLOAT_EQ(0.42F, point.intensity);
  }
}

TEST(LidarSynthesizerTest, IntensityThresholdDropsReturns)
{
  LidarSynthesizerConfig config = SmallConfig();
  config.intensity_threshold = 0.5;
  const LidarSynthesizer synthesizer(config);

  LidarSynthesizer::Box dim = MakeBox(15.0, 0.0, 1.0, 1.0, 1.0);
  dim.intensity = 0.1;

  EXPECT_TRUE(synthesizer.Synthesize({dim}).empty());
}

TEST(LidarSynthesizerTest, RangeGateDropsDistantBoxes)
{
  LidarSynthesizerConfig config = SmallConfig();
  config.max_range = 20.0;
  const LidarSynthesizer synthesizer(config);

  const pcl::PointCloud<pcl::PointXYZI> cloud = synthesizer.Synthesize(
    {MakeBox(60.0, 0.0, 1.0, 1.0, 1.0)});
  EXPECT_TRUE(cloud.empty());
}

TEST(LidarSynthesizerTest, NoisePerturbsPointsButStaysWithinBounds)
{
  LidarSynthesizerConfig config = SmallConfig();
  config.range_noise_std = 0.05;
  const LidarSynthesizer synthesizer(config);

  const pcl::PointCloud<pcl::PointXYZI> cloud = synthesizer.Synthesize(
    {MakeBox(15.0, 0.0, 1.0, 1.0, 1.0)});

  ASSERT_FALSE(cloud.empty());
  for (const auto & point : cloud.points) {
    const double range = std::hypot(
      static_cast<double>(point.x), static_cast<double>(point.y));
    // 4 sigma of headroom over the 13-17 m shell.
    EXPECT_GT(range, 12.5);
    EXPECT_LT(range, 17.5);
  }
}

TEST(LidarSynthesizerTest, TransformBoxToEgoFlipsTheLateralOffset)
{
  const FrameTransform transform(Pose{});
  const LidarSynthesizer::Box box = TransformBoxToEgo(
    transform, Eigen::Vector3d(20.0, 3.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0),
    0.0, 0.5, 7, "car");

  EXPECT_NEAR(20.0, box.center.x(), 1e-9);
  EXPECT_NEAR(-3.0, box.center.y(), 1e-9);
  EXPECT_EQ(7U, box.actor_id);
  EXPECT_EQ("car", box.class_label);
}

TEST(LidarSynthesizerTest, TransformBoxToEgoRecomputesYawThroughTheReflection)
{
  const FrameTransform transform(Pose{});
  const LidarSynthesizer::Box box = TransformBoxToEgo(
    transform, Eigen::Vector3d(20.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0),
    0.5, 0.5, 1, "car");

  // Under an identity ego pose the reflection alone negates the yaw.
  EXPECT_NEAR(-0.5, box.yaw, 1e-9);
}

TEST(LidarSynthesizerTest, ConfigIsExposedForDiagnostics)
{
  const LidarSynthesizerConfig config = SmallConfig();
  const LidarSynthesizer synthesizer(config);
  EXPECT_EQ(config.channels, synthesizer.config().channels);
  EXPECT_DOUBLE_EQ(config.max_range, synthesizer.config().max_range);
}

}  // namespace
}  // namespace sil_carla_bridge
