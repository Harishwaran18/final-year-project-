// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_SENSOR_FUSION__SENSOR_FUSION_NODE_HPP_
#define SIL_SENSOR_FUSION__SENSOR_FUSION_NODE_HPP_

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "sil_interfaces/msg/detection2_d_array.hpp"
#include "sil_interfaces/msg/obstacle_array.hpp"
#include "sil_sensor_fusion/camera_projection.hpp"
#include "sil_sensor_fusion/lidar_clusterer.hpp"
#include "sil_sensor_fusion/multi_target_tracker.hpp"

namespace sil_sensor_fusion
{

/// ROS 2 wrapper around the LiDAR/camera fusion pipeline.
///
/// Subscribes to a CARLA-derived PointXYZI sweep and a 2D detection array,
/// runs ground removal + clustering + EKF tracking, and publishes fused
/// obstacle tracks plus a MarkerArray for RViz.
class SensorFusionNode : public rclcpp::Node
{
public:
  explicit SensorFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  /// Number of fusion cycles completed; used by the launch test.
  std::uint64_t cycleCount() const {return cycle_count_;}

  /// Last measured fusion latency [ms].
  double lastLatencyMs() const {return last_latency_ms_;}

private:
  /// LiDAR sweep callback: clusters the sweep and feeds polar observations.
  void OnPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /// Camera detection callback: caches bearings/ranges for the next cycle.
  void OnDetections(sil_interfaces::msg::Detection2DArray::ConstSharedPtr msg);

  /// Runs one fusion cycle and publishes the resulting tracks.
  void RunCycle(const std_msgs::msg::Header & header);

  void DeclareParameters();
  void LoadParameters();

  LidarClusterer clusterer_;
  MultiTargetTracker tracker_;
  CameraIntrinsics intrinsics_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sil_interfaces::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<sil_interfaces::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::vector<CameraObservation> pending_camera_observations_;
  std::uint64_t cycle_count_{0};
  double last_latency_ms_{0.0};

  // Parameters.
  double min_range_{1.5};
  double max_range_{80.0};
  double cluster_tolerance_{0.65};
  std::uint32_t confirm_hits_{3};
  double process_noise_std_{1.5};
  double range_noise_std_{0.25};
  double bearing_noise_std_{0.02};
  double association_gate_{9.2103};
  bool publish_markers_{true};
  bool use_camera_observations_{true};
};

}  // namespace sil_sensor_fusion

#endif  // SIL_SENSOR_FUSION__SENSOR_FUSION_NODE_HPP_
