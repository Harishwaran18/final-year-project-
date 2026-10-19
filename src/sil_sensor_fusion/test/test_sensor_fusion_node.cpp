// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sil_interfaces/msg/detection2_d.hpp"
#include "sil_interfaces/msg/detection2_d_array.hpp"
#include "sil_interfaces/msg/obstacle_array.hpp"
#include "sil_sensor_fusion/sensor_fusion_node.hpp"

namespace sil_sensor_fusion
{
namespace
{

using namespace std::chrono_literals;

/// Node-level tests for the fusion node.
///
/// The EKF and clustering maths are covered by the pure suites; what these
/// check is the ROS plumbing, the parameter surface, and that a synthetic sweep
/// actually reaches the output topic.
class SensorFusionNodeTest : public ::testing::Test
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
      return "/sil_sensor_fusion_test";
    }
    return std::string("/") + info->test_suite_name() + "_" + info->name();
  }

  /// Adds this test's private namespace to @p options.
  static rclcpp::NodeOptions Isolate(rclcpp::NodeOptions options)
  {
    options.arguments({"--ros-args", "-r", "__ns:=" + NamespaceForCurrentTest()});
    return options;
  }

  static rclcpp::NodeOptions DefaultOptions()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"confirm_hits", 2},
      {"publish_markers", false},
    });
    return Isolate(options);
  }

  /// A sweep with a ground plane and one vehicle-sized block 20 m ahead.
  static pcl::PointCloud<pcl::PointXYZI> MakeSweep()
  {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for (int x = 0; x < 40; ++x) {
      for (int y = -20; y <= 20; ++y) {
        pcl::PointXYZI point;
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y) * 0.5F;
        point.z = -1.5F;
        point.intensity = 0.1F;
        cloud.push_back(point);
      }
    }
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        for (int k = 0; k < 6; ++k) {
          pcl::PointXYZI point;
          point.x = static_cast<float>(19.1 + 1.8 * i / 5.0);
          point.y = static_cast<float>(-0.9 + 1.8 * j / 5.0);
          point.z = static_cast<float>(-0.75 + 1.5 * k / 5.0);
          point.intensity = 0.8F;
          cloud.push_back(point);
        }
      }
    }
    return cloud;
  }
};

TEST_F(SensorFusionNodeTest, ConstructsAndDeclaresItsParameters)
{
  auto node = std::make_shared<SensorFusionNode>(DefaultOptions());
  EXPECT_EQ(0U, node->cycleCount());
  EXPECT_DOUBLE_EQ(0.0, node->lastLatencyMs());
}

TEST_F(SensorFusionNodeTest, ParametersOverrideTheClusterConfig)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"min_range", 5.0},
    {"max_range", 40.0},
    {"cluster_tolerance", 0.9},
    {"confirm_hits", 4},
  });

  auto node = std::make_shared<SensorFusionNode>(Isolate(options));
  EXPECT_DOUBLE_EQ(5.0, node->get_parameter("min_range").as_double());
  EXPECT_DOUBLE_EQ(40.0, node->get_parameter("max_range").as_double());
  EXPECT_EQ(4, node->get_parameter("confirm_hits").as_int());
}

TEST_F(SensorFusionNodeTest, ASweepProducesATrackOnTheOutputTopic)
{
  auto node = std::make_shared<SensorFusionNode>(DefaultOptions());

  sil_interfaces::msg::ObstacleArray::SharedPtr received;
  auto subscriber = node->create_subscription<sil_interfaces::msg::ObstacleArray>(
    "fusion/obstacles", rclcpp::QoS(10),
    [&received](sil_interfaces::msg::ObstacleArray::ConstSharedPtr msg) {received =
      std::make_shared<sil_interfaces::msg::ObstacleArray>(*msg);});

  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "lidar/points", rclcpp::SensorDataQoS());

  SpinFor(node, 300ms);

  pcl::PointCloud<pcl::PointXYZI> cloud = MakeSweep();
  for (int cycle = 0; cycle < 6; ++cycle) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = node->now();
    msg.header.frame_id = "base_link";
    publisher->publish(msg);
    SpinFor(node, 60ms);
  }

  ASSERT_NE(nullptr, received) << "the fusion node must publish an ObstacleArray";
  EXPECT_GE(node->cycleCount(), 1U);
  EXPECT_GT(received->cycle_id, 0U);
  ASSERT_FALSE(received->obstacles.empty())
    << "the synthetic vehicle must be tracked";
  EXPECT_NEAR(20.0, received->obstacles.front().position.x, 2.0);
}

TEST_F(SensorFusionNodeTest, LatencyIsMeasuredPerCycle)
{
  auto node = std::make_shared<SensorFusionNode>(DefaultOptions());

  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "lidar/points", rclcpp::SensorDataQoS());
  SpinFor(node, 200ms);

  pcl::PointCloud<pcl::PointXYZI> cloud = MakeSweep();
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = node->now();
  msg.header.frame_id = "base_link";
  publisher->publish(msg);
  SpinFor(node, 300ms);

  ASSERT_GE(node->cycleCount(), 1U);
  // The budget is 18 ms; a wildly larger value means the measurement is broken
  // (e.g. it is timing something other than the fusion step).
  EXPECT_LT(node->lastLatencyMs(), 1000.0);
}

TEST_F(SensorFusionNodeTest, CameraDetectionsDoNotBreakTheCycle)
{
  auto node = std::make_shared<SensorFusionNode>(DefaultOptions());

  auto detection_pub = node->create_publisher<sil_interfaces::msg::Detection2DArray>(
    "camera/detections", rclcpp::SensorDataQoS());
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "lidar/points", rclcpp::SensorDataQoS());
  SpinFor(node, 200ms);

  sil_interfaces::msg::Detection2DArray detections;
  detections.fx = 1100.0;
  detections.fy = 1100.0;
  detections.cx = 640.0;
  detections.cy = 360.0;
  detections.image_width = 1280;
  detections.image_height = 720;

  sil_interfaces::msg::Detection2D detection;
  detection.class_label = "car";
  detection.x_offset = 600;
  detection.y_offset = 300;
  detection.width = 120;
  detection.height = 150;
  detections.detections.push_back(detection);

  pcl::PointCloud<pcl::PointXYZI> cloud = MakeSweep();
  for (int cycle = 0; cycle < 4; ++cycle) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = node->now();
    msg.header.frame_id = "base_link";

    detection_pub->publish(detections);
    publisher->publish(msg);
    SpinFor(node, 60ms);
  }

  EXPECT_GE(node->cycleCount(), 1U)
    << "camera observations must not stall the fusion cycle";
}

}  // namespace
}  // namespace sil_sensor_fusion
