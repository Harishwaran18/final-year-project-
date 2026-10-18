// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "sil_can_gateway/can_gateway_node.hpp"
#include "sil_interfaces/msg/safety_alert.hpp"
#include "std_msgs/msg/float32.hpp"

namespace sil_can_gateway
{
namespace
{

using namespace std::chrono_literals;

/// Node-level tests for the CAN gateway.
///
/// These always run in dry-run mode so they never depend on a `vcan0`
/// interface existing: the point is the encoding and publish/subscribe wiring,
/// which is where a regression would silently stop braking commands.
class CanGatewayNodeTest : public ::testing::Test
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
      return "/sil_can_gateway_test";
    }
    return std::string("/") + info->test_suite_name() + "_" + info->name();
  }

  /// Adds this test's private namespace to @p options.
  static rclcpp::NodeOptions Isolate(rclcpp::NodeOptions options)
  {
    options.arguments({"--ros-args", "-r", "__ns:=" + NamespaceForCurrentTest()});
    return options;
  }

  static rclcpp::NodeOptions DryRunOptions(double rate_hz = 200.0)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"dry_run", true},
      {"transmit_rate_hz", rate_hz},
      {"alert_timeout", 0.2},
    });
    return Isolate(options);
  }
};

TEST_F(CanGatewayNodeTest, ConstructsInDryRunMode)
{
  auto node = std::make_shared<CanGatewayNode>(DryRunOptions());
  EXPECT_FALSE(node->isConnected()) << "dry-run must not touch a real interface";
  EXPECT_EQ(0U, node->transmitCount());
}

TEST_F(CanGatewayNodeTest, TimerEncodesFramesWithoutAnInterface)
{
  auto node = std::make_shared<CanGatewayNode>(DryRunOptions());
  SpinFor(node, 150ms);

  EXPECT_GT(node->transmitCount(), 0U)
    << "the gateway must keep encoding even when no bus is present";
  EXPECT_EQ(protocol::kBrakeCommandId, node->lastFrame().can_id);
  EXPECT_EQ(protocol::kDlc, node->lastFrame().dlc);
}

TEST_F(CanGatewayNodeTest, EncodesAReceivedBrakeAlert)
{
  auto node = std::make_shared<CanGatewayNode>(DryRunOptions());

  auto alert_pub = node->create_publisher<sil_interfaces::msg::SafetyAlert>("aeb/alert", 10);
  SpinFor(node, 100ms);

  sil_interfaces::msg::SafetyAlert alert;
  alert.state = static_cast<std::uint8_t>(3);
  alert.target_track_id = 12;
  alert.requested_deceleration = 8.0;
  alert.brake_pressure = 1.0;
  alert.state_changed = true;

  for (int i = 0; i < 20; ++i) {
    alert_pub->publish(alert);
    SpinFor(node, 20ms);
  }

  const CanFrame frame = node->lastFrame();
  ASSERT_EQ(protocol::kBrakeCommandId, frame.can_id);

  BrakeCommand decoded;
  ASSERT_TRUE(DecodeBrakeCommand(frame, decoded));
  EXPECT_EQ(3, decoded.state);
  EXPECT_EQ(12U, decoded.target_track_id);
  EXPECT_NEAR(8.0, decoded.requested_deceleration, protocol::kDecelerationScale);
  EXPECT_NEAR(1.0, decoded.brake_pressure, protocol::kPressureScale);
}

TEST_F(CanGatewayNodeTest, StaleAlertsFallBackToAZeroBrakeCommand)
{
  auto node = std::make_shared<CanGatewayNode>(DryRunOptions());

  auto alert_pub = node->create_publisher<sil_interfaces::msg::SafetyAlert>("aeb/alert", 10);
  SpinFor(node, 100ms);

  sil_interfaces::msg::SafetyAlert alert;
  alert.state = static_cast<std::uint8_t>(3);
  alert.requested_deceleration = 9.0;
  alert.brake_pressure = 1.0;

  for (int i = 0; i < 20; ++i) {
    alert_pub->publish(alert);
    SpinFor(node, 20ms);
  }

  {
    BrakeCommand decoded;
    ASSERT_TRUE(DecodeBrakeCommand(node->lastFrame(), decoded));
    ASSERT_GT(decoded.brake_pressure, 0.0);
  }

  // After the alert timeout the gateway must release the brakes; latching the
  // last command would be the unsafe failure mode.
  SpinFor(node, 600ms);

  BrakeCommand decoded;
  ASSERT_TRUE(DecodeBrakeCommand(node->lastFrame(), decoded));
  EXPECT_EQ(0, decoded.state);
  EXPECT_DOUBLE_EQ(0.0, decoded.brake_pressure);
}

TEST_F(CanGatewayNodeTest, ConnectReportsFailureForAMissingInterface)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"dry_run", true},
    {"interface_name", "sil_no_such_can0"},
  });
  auto node = std::make_shared<CanGatewayNode>(Isolate(options));

  // An explicit Connect() attempt against a missing interface must report false
  // rather than throw, so the node can stay up.
  EXPECT_FALSE(node->Connect());
  EXPECT_FALSE(node->isConnected());
}

TEST_F(CanGatewayNodeTest, PublishAlertTopicIsSubscribed)
{
  auto node = std::make_shared<CanGatewayNode>(DryRunOptions());

  // A publisher with a matching QoS must reach the node's subscription.
  auto alert_pub = node->create_publisher<sil_interfaces::msg::SafetyAlert>("aeb/alert", 10);
  SpinFor(node, 200ms);

  EXPECT_GE(alert_pub->get_subscription_count(), 1U)
    << "the gateway must subscribe to aeb/alert";
}

}  // namespace
}  // namespace sil_can_gateway
