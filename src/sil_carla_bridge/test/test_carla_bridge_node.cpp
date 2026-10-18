// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "sil_carla_bridge/carla_bridge_node.hpp"
#include "sil_interfaces/msg/carla_actor.hpp"
#include "sil_interfaces/msg/carla_actor_array.hpp"
#include "sil_interfaces/msg/detection2_d_array.hpp"
#include "std_msgs/msg/float32.hpp"

namespace sil_carla_bridge
{
namespace
{

using namespace std::chrono_literals;

/// Node-level tests for the CARLA bridge's synthesis path.
///
/// These deliberately avoid a running simulator: they drive ProcessActors()
/// directly, which is exactly what the relay mode feeds it, and assert on the
/// counters and the published messages.
class CarlaBridgeNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static void SpinFor(
    const std::shared_ptr<rclcpp::Node> & node, std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(2ms);
    }
  }

  /// Topics are resolved relative to the node namespace. Each test gets its own
  /// namespace so that test binaries running concurrently under `colcon test`
  /// cannot observe each other's publications on identically named topics.
  static std::string NamespaceForCurrentTest()
  {
    const ::testing::TestInfo * info =
      ::testing::UnitTest::GetInstance()->current_test_info();
    if (info == nullptr) {
      return "/sil_carla_bridge_test";
    }
    return std::string("/") + info->test_suite_name() + "_" + info->name();
  }

  /// Adds this test's private namespace to @p options.
  static rclcpp::NodeOptions Isolate(rclcpp::NodeOptions options)
  {
    options.arguments({"--ros-args", "-r", "__ns:=" + NamespaceForCurrentTest()});
    return options;
  }

  static rclcpp::NodeOptions SynthesisOptions()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"synthesis_mode", true},
      {"lidar_channels", 8},
      {"lidar_azimuth_samples", 128},
      {"lidar_range_noise_std", 0.0},
      {"camera_image_width", 1280},
      {"camera_image_height", 720},
    });
    return Isolate(options);
  }

  /// One vehicle 20 m directly ahead of the ego, in the CARLA frame.
  static sil_interfaces::msg::CarlaActorArray MakeActorArray()
  {
    sil_interfaces::msg::CarlaActorArray actors;
    actors.header.stamp = rclcpp::Clock().now();
    actors.ego_position.x = 0.0;
    actors.ego_position.y = 0.0;
    actors.ego_position.z = 0.0;
    actors.ego_rotation.z = 0.0;
    actors.ego_velocity.x = 15.0;

    sil_interfaces::msg::CarlaActor actor;
    actor.actor_id = 42;
    actor.class_label = "vehicle.tesla.model3";
    actor.position.x = 20.0;
    actor.position.y = 0.0;
    actor.position.z = 0.75;
    actor.extent.x = 2.4;
    actor.extent.y = 1.0;
    actor.extent.z = 0.75;
    actors.actors.push_back(actor);
    return actors;
  }
};

TEST_F(CarlaBridgeNodeTest, ConstructsWithNoActors)
{
  auto node = std::make_shared<CarlaBridgeNode>(SynthesisOptions());
  EXPECT_EQ(0U, node->actorMessageCount());
  EXPECT_EQ(0U, node->synthesizedSweepCount());
  EXPECT_EQ(0U, node->synthesizedFrameCount());
}

TEST_F(CarlaBridgeNodeTest, ProcessActorsPublishesASweepAndDetections)
{
  auto node = std::make_shared<CarlaBridgeNode>(SynthesisOptions());

  node->ProcessActors(MakeActorArray());

  EXPECT_EQ(1U, node->actorMessageCount());
  EXPECT_EQ(1U, node->synthesizedSweepCount());
  EXPECT_EQ(1U, node->synthesizedFrameCount());
}

TEST_F(CarlaBridgeNodeTest, CountersAccumulateAcrossTicks)
{
  auto node = std::make_shared<CarlaBridgeNode>(SynthesisOptions());
  const auto actors = MakeActorArray();

  for (int tick = 0; tick < 5; ++tick) {
    node->ProcessActors(actors);
  }

  EXPECT_EQ(5U, node->actorMessageCount());
  EXPECT_EQ(5U, node->synthesizedSweepCount());
  EXPECT_EQ(5U, node->synthesizedFrameCount());
}

TEST_F(CarlaBridgeNodeTest, SynthesisPublishesAPointCloud)
{
  auto node = std::make_shared<CarlaBridgeNode>(SynthesisOptions());

  sensor_msgs::msg::PointCloud2::SharedPtr received;
  auto subscriber = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "lidar/points", rclcpp::SensorDataQoS(),
    [&received](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {received =
      std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);});

  SpinFor(node, 200ms);
  node->ProcessActors(MakeActorArray());
  SpinFor(node, 300ms);

  ASSERT_NE(nullptr, received) << "a synthetic sweep must be published";
  EXPECT_GT(received->width, 0U);
  EXPECT_EQ("base_link", received->header.frame_id);
}

TEST_F(CarlaBridgeNodeTest, SynthesisPublishesDetectionsWithIntrinsics)
{
  auto node = std::make_shared<CarlaBridgeNode>(SynthesisOptions());

  sil_interfaces::msg::Detection2DArray::SharedPtr received;
  auto subscriber = node->create_subscription<sil_interfaces::msg::Detection2DArray>(
    "camera/detections", rclcpp::SensorDataQoS(),
    [&received](sil_interfaces::msg::Detection2DArray::ConstSharedPtr msg) {received =
      std::make_shared<sil_interfaces::msg::Detection2DArray>(*msg);});

  SpinFor(node, 200ms);
  node->ProcessActors(MakeActorArray());
  SpinFor(node, 300ms);

  ASSERT_NE(nullptr, received) << "synthetic detections must be published";
  EXPECT_EQ(1280U, received->image_width);
  EXPECT_EQ(720U, received->image_height);
  EXPECT_GT(received->fx, 0.0);
  ASSERT_FALSE(received->detections.empty());
  EXPECT_EQ("car", received->detections.front().class_label);
  EXPECT_GT(received->detections.front().height, 0U);
}

TEST_F(CarlaBridgeNodeTest, EgoSpeedIsPublishedFromTheEgoVelocity)
{
  auto node = std::make_shared<CarlaBridgeNode>(SynthesisOptions());

  std_msgs::msg::Float32::SharedPtr received;
  auto subscriber = node->create_subscription<std_msgs::msg::Float32>(
    "vehicle/ego_speed", rclcpp::QoS(10),
    [&received](std_msgs::msg::Float32::ConstSharedPtr msg) {received =
      std::make_shared<std_msgs::msg::Float32>(*msg);});

  SpinFor(node, 200ms);
  node->ProcessActors(MakeActorArray());
  SpinFor(node, 300ms);

  ASSERT_NE(nullptr, received);
  EXPECT_NEAR(15.0F, received->data, 1e-3);
}

TEST_F(CarlaBridgeNodeTest, AnActorInsideTheEgoFootprintIsDropped)
{
  auto node = std::make_shared<CarlaBridgeNode>(SynthesisOptions());

  sil_interfaces::msg::CarlaActorArray actors = MakeActorArray();
  actors.actors.front().position.x = 0.5;  // inside the 1.5 m gate

  sil_interfaces::msg::Detection2DArray::SharedPtr received;
  auto subscriber = node->create_subscription<sil_interfaces::msg::Detection2DArray>(
    "camera/detections", rclcpp::SensorDataQoS(),
    [&received](sil_interfaces::msg::Detection2DArray::ConstSharedPtr msg) {received =
      std::make_shared<sil_interfaces::msg::Detection2DArray>(*msg);});

  SpinFor(node, 200ms);
  node->ProcessActors(actors);
  SpinFor(node, 300ms);

  ASSERT_NE(nullptr, received);
  EXPECT_TRUE(received->detections.empty())
    << "an actor overlapping the ego must not produce a detection";
}

TEST_F(CarlaBridgeNodeTest, RelayModeSkipsSynthesis)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"synthesis_mode", false}});
  auto node = std::make_shared<CarlaBridgeNode>(Isolate(options));

  node->ProcessActors(MakeActorArray());

  EXPECT_EQ(1U, node->actorMessageCount());
  EXPECT_EQ(0U, node->synthesizedSweepCount())
    << "relay mode must leave the sensor topics to CARLA itself";
  EXPECT_EQ(0U, node->synthesizedFrameCount());
}

}  // namespace
}  // namespace sil_carla_bridge
