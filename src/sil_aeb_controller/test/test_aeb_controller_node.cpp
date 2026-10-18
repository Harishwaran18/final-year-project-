// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "sil_aeb_controller/aeb_controller_node.hpp"
#include "sil_interfaces/msg/obstacle_array.hpp"
#include "sil_interfaces/srv/set_aeb_state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace sil_aeb_controller
{
namespace
{

using namespace std::chrono_literals;

/// Topics and services are resolved relative to the node namespace. Every test
/// gets its own namespace so that concurrently running test binaries cannot see
/// each other's messages, which is what `colcon test` would otherwise cause.
std::string NamespaceForCurrentTest()
{
  const ::testing::TestInfo * info =
    ::testing::UnitTest::GetInstance()->current_test_info();
  if (info == nullptr) {
    return "/sil_aeb_controller_test";
  }
  return std::string("/") + info->test_suite_name() + "_" + info->name();
}

/// Node-level smoke and wiring tests.
///
/// These run on a real rclcpp executor rather than mocks: the FSM itself is
/// covered by the pure-math suites, so what is exercised here is the ROS
/// plumbing (parameter declaration, subscriptions, the service, the timer).
class AebControllerNodeTest : public ::testing::Test
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

  /// Spins @p node for @p duration so callbacks and timers can fire.
  static void SpinFor(
    const std::shared_ptr<rclcpp::Node> & node, std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(2ms);
    }
  }

  /// Adds this test's private namespace to @p options.
  static rclcpp::NodeOptions Isolate(rclcpp::NodeOptions options)
  {
    options.arguments({"--ros-args", "-r", "__ns:=" + NamespaceForCurrentTest()});
    return options;
  }

  static rclcpp::NodeOptions FastOptions()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"loop_rate_hz", 200.0},
      {"obstacle_timeout", 0.2},
    });
    return Isolate(options);
  }
};

TEST_F(AebControllerNodeTest, ConstructsAndDeclaresItsParameters)
{
  auto node = std::make_shared<AebControllerNode>(FastOptions());
  EXPECT_EQ(0U, node->cycleCount());
  EXPECT_EQ(AebState::kNormal, node->lastOutput().state);
}

TEST_F(AebControllerNodeTest, TimerRunsTheFsmAtTheConfiguredRate)
{
  auto node = std::make_shared<AebControllerNode>(FastOptions());
  SpinFor(node, 150ms);

  EXPECT_GT(node->cycleCount(), 0U)
    << "the control timer must drive the FSM";
}

TEST_F(AebControllerNodeTest, ObstaclesAndSpeedDriveAnEscalation)
{
  auto node = std::make_shared<AebControllerNode>(FastOptions());

  auto speed_pub = node->create_publisher<std_msgs::msg::Float32>("vehicle/ego_speed", 10);
  auto obstacle_pub = node->create_publisher<sil_interfaces::msg::ObstacleArray>(
    "fusion/obstacles", 10);

  // Wait for the node's subscriptions to be matched before publishing.
  SpinFor(node, 100ms);

  std_msgs::msg::Float32 speed;
  speed.data = 20.0F;
  speed_pub->publish(speed);

  sil_interfaces::msg::ObstacleArray obstacles;
  sil_interfaces::msg::Obstacle obstacle;
  obstacle.track_id = 7;
  obstacle.position.x = 8.0;
  obstacle.position.y = 0.0;
  obstacle.velocity.x = -20.0;
  obstacle.closing_speed = 20.0;
  obstacle.range = 8.0;
  obstacles.obstacles.push_back(obstacle);

  for (int i = 0; i < 30; ++i) {
    obstacle_pub->publish(obstacles);
    speed_pub->publish(speed);
    SpinFor(node, 20ms);
  }

  const AebOutput output = node->lastOutput();
  EXPECT_NE(AebState::kNormal, output.state)
    << "a closing target at 8 m must escalate the FSM";
  EXPECT_EQ(7U, output.target_track_id);
  EXPECT_GT(output.brake_pressure, 0.0);
}

TEST_F(AebControllerNodeTest, StaleObstaclesFallBackToNormal)
{
  auto node = std::make_shared<AebControllerNode>(FastOptions());

  auto speed_pub = node->create_publisher<std_msgs::msg::Float32>("vehicle/ego_speed", 10);
  auto obstacle_pub = node->create_publisher<sil_interfaces::msg::ObstacleArray>(
    "fusion/obstacles", 10);
  SpinFor(node, 100ms);

  std_msgs::msg::Float32 speed;
  speed.data = 20.0F;

  sil_interfaces::msg::ObstacleArray obstacles;
  sil_interfaces::msg::Obstacle obstacle;
  obstacle.track_id = 3;
  obstacle.position.x = 6.0;
  obstacle.velocity.x = -20.0;
  obstacle.closing_speed = 20.0;
  obstacle.range = 6.0;
  obstacles.obstacles.push_back(obstacle);

  for (int i = 0; i < 20; ++i) {
    obstacle_pub->publish(obstacles);
    speed_pub->publish(speed);
    SpinFor(node, 20ms);
  }
  ASSERT_NE(AebState::kNormal, node->lastOutput().state);

  // Stop publishing: after the 0.2 s timeout the tracks must be dropped and the
  // FSM must release the brakes.
  SpinFor(node, 600ms);
  EXPECT_EQ(AebState::kNormal, node->lastOutput().state);
  EXPECT_DOUBLE_EQ(0.0, node->lastOutput().brake_pressure);
}

TEST_F(AebControllerNodeTest, SetStateServiceLatchesAnOverride)
{
  auto node = std::make_shared<AebControllerNode>(FastOptions());

  auto client = node->create_client<sil_interfaces::srv::SetAebState>("aeb/set_state");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<sil_interfaces::srv::SetAebState::Request>();
  request->state = static_cast<std::uint8_t>(AebState::kFullEmergencyBrake);
  request->latch = true;

  auto future = client->async_send_request(request);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    SpinFor(node, 20ms);
  }

  ASSERT_EQ(std::future_status::ready, future.wait_for(0ms));
  const auto response = future.get();
  EXPECT_TRUE(response->success);
  EXPECT_EQ(static_cast<std::uint8_t>(AebState::kFullEmergencyBrake), response->active_state);

  SpinFor(node, 100ms);
  EXPECT_EQ(AebState::kFullEmergencyBrake, node->lastOutput().state);
  EXPECT_TRUE(node->lastOutput().overridden);
}

TEST_F(AebControllerNodeTest, SetStateServiceRejectsAnOutOfRangeState)
{
  auto node = std::make_shared<AebControllerNode>(FastOptions());

  auto client = node->create_client<sil_interfaces::srv::SetAebState>("aeb/set_state");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto request = std::make_shared<sil_interfaces::srv::SetAebState::Request>();
  request->state = 99;

  auto future = client->async_send_request(request);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    SpinFor(node, 20ms);
  }

  ASSERT_EQ(std::future_status::ready, future.wait_for(0ms));
  const auto response = future.get();
  EXPECT_FALSE(response->success);
}

TEST_F(AebControllerNodeTest, SetStateServiceReleasesAnOverride)
{
  auto node = std::make_shared<AebControllerNode>(FastOptions());

  auto client = node->create_client<sil_interfaces::srv::SetAebState>("aeb/set_state");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto latch = std::make_shared<sil_interfaces::srv::SetAebState::Request>();
  latch->state = static_cast<std::uint8_t>(AebState::kPartialBraking);
  latch->latch = true;

  auto future = client->async_send_request(latch);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < deadline)
  {
    SpinFor(node, 20ms);
  }
  ASSERT_EQ(std::future_status::ready, future.wait_for(0ms));
  future.get();
  SpinFor(node, 60ms);
  EXPECT_EQ(AebState::kPartialBraking, node->lastOutput().state);

  auto release = std::make_shared<sil_interfaces::srv::SetAebState::Request>();
  release->state = static_cast<std::uint8_t>(AebState::kNormal);
  release->latch = false;

  auto release_future = client->async_send_request(release);
  const auto release_deadline = std::chrono::steady_clock::now() + 3s;
  while (release_future.wait_for(0ms) != std::future_status::ready &&
    std::chrono::steady_clock::now() < release_deadline)
  {
    SpinFor(node, 20ms);
  }
  ASSERT_EQ(std::future_status::ready, release_future.wait_for(0ms));
  release_future.get();

  SpinFor(node, 100ms);
  EXPECT_EQ(AebState::kNormal, node->lastOutput().state);
  EXPECT_FALSE(node->lastOutput().overridden);
}

TEST_F(AebControllerNodeTest, ParametersOverrideTheThresholds)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"warning_ttc", 4.0},
    {"partial_braking_ttc", 3.0},
    {"full_braking_ttc", 1.0},
  });
  auto node = std::make_shared<AebControllerNode>(Isolate(options));

  // A 3.5 s TTC would warn under the defaults but is NORMAL at 4.0 s.
  auto speed_pub = node->create_publisher<std_msgs::msg::Float32>("vehicle/ego_speed", 10);
  auto obstacle_pub = node->create_publisher<sil_interfaces::msg::ObstacleArray>(
    "fusion/obstacles", 10);
  SpinFor(node, 100ms);

  std_msgs::msg::Float32 speed;
  speed.data = 10.0F;

  sil_interfaces::msg::ObstacleArray obstacles;
  sil_interfaces::msg::Obstacle obstacle;
  obstacle.track_id = 1;
  obstacle.position.x = 50.0;
  obstacle.closing_speed = 10.0;
  obstacle.range = 50.0;
  obstacles.obstacles.push_back(obstacle);

  for (int i = 0; i < 20; ++i) {
    obstacle_pub->publish(obstacles);
    speed_pub->publish(speed);
    SpinFor(node, 20ms);
  }

  // TTC = 5 s, above the overridden 4 s warning threshold.
  EXPECT_EQ(AebState::kNormal, node->lastOutput().state);
}

}  // namespace
}  // namespace sil_aeb_controller
