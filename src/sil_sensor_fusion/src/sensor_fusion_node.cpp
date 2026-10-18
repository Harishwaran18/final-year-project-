// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_sensor_fusion/sensor_fusion_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sil_sensor_fusion
{
namespace
{
using SteadyClock = std::chrono::steady_clock;

/// Colour ramp shared with the RViz markers: green -> yellow -> red as the
/// closing speed rises.
void ClosingSpeedColour(double closing_speed, float & r, float & g, float & b)
{
  const double t = std::clamp(closing_speed / 30.0, 0.0, 1.0);
  r = static_cast<float>(t);
  g = static_cast<float>(1.0 - t);
  b = 0.05F;
}
}  // namespace

SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("sil_sensor_fusion", options)
{
  DeclareParameters();
  LoadParameters();

  clusterer_ = LidarClusterer(ClusterConfig{});
  tracker_ = MultiTargetTracker(
    TrackerConfig{confirm_hits_, 4, association_gate_, 6.0},
    EkfConfig{process_noise_std_, range_noise_std_, bearing_noise_std_, association_gate_});

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "lidar/points", rclcpp::SensorDataQoS(),
    std::bind(&SensorFusionNode::OnPointCloud, this, std::placeholders::_1));

  detection_sub_ = create_subscription<sil_interfaces::msg::Detection2DArray>(
    "camera/detections", rclcpp::SensorDataQoS(),
    std::bind(&SensorFusionNode::OnDetections, this, std::placeholders::_1));

  obstacle_pub_ = create_publisher<sil_interfaces::msg::ObstacleArray>(
    "fusion/obstacles", rclcpp::QoS(10));

  if (publish_markers_) {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "fusion/obstacle_markers", rclcpp::QoS(10));
  }

  RCLCPP_INFO(get_logger(),
    "sil_sensor_fusion ready: range=[%.1f, %.1f] m, tolerance=%.2f m, confirm_hits=%u",
    min_range_, max_range_, cluster_tolerance_, confirm_hits_);
}

void SensorFusionNode::DeclareParameters()
{
  declare_parameter("min_range", min_range_);
  declare_parameter("max_range", max_range_);
  declare_parameter("cluster_tolerance", cluster_tolerance_);
  declare_parameter("confirm_hits", static_cast<int>(confirm_hits_));
  declare_parameter("process_noise_std", process_noise_std_);
  declare_parameter("range_noise_std", range_noise_std_);
  declare_parameter("bearing_noise_std", bearing_noise_std_);
  declare_parameter("association_gate", association_gate_);
  declare_parameter("publish_markers", publish_markers_);
  declare_parameter("use_camera_observations", use_camera_observations_);
  declare_parameter("ground_distance_threshold", 0.12);
  declare_parameter("min_cluster_size", 8);
  declare_parameter("max_cluster_size", 4000);
  declare_parameter("min_height", -1.8);
  declare_parameter("max_height", 3.0);
}

void SensorFusionNode::LoadParameters()
{
  min_range_ = get_parameter("min_range").as_double();
  max_range_ = get_parameter("max_range").as_double();
  cluster_tolerance_ = get_parameter("cluster_tolerance").as_double();
  confirm_hits_ = static_cast<std::uint32_t>(get_parameter("confirm_hits").as_int());
  process_noise_std_ = get_parameter("process_noise_std").as_double();
  range_noise_std_ = get_parameter("range_noise_std").as_double();
  bearing_noise_std_ = get_parameter("bearing_noise_std").as_double();
  association_gate_ = get_parameter("association_gate").as_double();
  publish_markers_ = get_parameter("publish_markers").as_bool();
  use_camera_observations_ = get_parameter("use_camera_observations").as_bool();

  ClusterConfig cluster_config;
  cluster_config.min_range = min_range_;
  cluster_config.max_range = max_range_;
  cluster_config.cluster_tolerance = cluster_tolerance_;
  cluster_config.ground_distance_threshold = get_parameter("ground_distance_threshold").as_double();
  cluster_config.min_cluster_size =
    static_cast<std::size_t>(get_parameter("min_cluster_size").as_int());
  cluster_config.max_cluster_size =
    static_cast<std::size_t>(get_parameter("max_cluster_size").as_int());
  cluster_config.min_height = get_parameter("min_height").as_double();
  cluster_config.max_height = get_parameter("max_height").as_double();
  clusterer_ = LidarClusterer(cluster_config);
}

void SensorFusionNode::OnDetections(
  sil_interfaces::msg::Detection2DArray::ConstSharedPtr msg)
{
  intrinsics_.fx = msg->fx;
  intrinsics_.fy = msg->fy;
  intrinsics_.cx = msg->cx;
  intrinsics_.cy = msg->cy;
  intrinsics_.image_width = msg->image_width;
  intrinsics_.image_height = msg->image_height;

  pending_camera_observations_.clear();
  if (!use_camera_observations_ || !intrinsics_.valid()) {
    return;
  }

  pending_camera_observations_.reserve(msg->detections.size());
  for (const auto & detection : msg->detections) {
    BoundingBox2D box;
    box.x_offset = detection.x_offset;
    box.y_offset = detection.y_offset;
    box.width = detection.width;
    box.height = detection.height;
    box.class_label = detection.class_label;

    CameraObservation observation = MakeCameraObservation(box, intrinsics_);
    if (observation.range_valid) {
      pending_camera_observations_.push_back(observation);
    }
  }
}

void SensorFusionNode::OnPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const auto start = SteadyClock::now();

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(*msg, cloud);

  const std::vector<Cluster> clusters = clusterer_.ExtractClusters(cloud);

  std::vector<PolarObservation> observations;
  observations.reserve(clusters.size() + pending_camera_observations_.size());

  for (const auto & cluster : clusters) {
    PolarObservation observation;
    observation.range = cluster.Range();
    observation.bearing = cluster.Bearing();
    observation.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    observation.modality = PolarObservation::Modality::kLidar;
    observations.push_back(observation);
  }

  // Camera observations augment the LiDAR clusters; the tracker's association
  // gate decides whether they refine an existing track or start a new one.
  for (const auto & camera_observation : pending_camera_observations_) {
    PolarObservation observation;
    observation.range = camera_observation.range;
    observation.bearing = camera_observation.bearing;
    observation.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    observation.modality = PolarObservation::Modality::kCamera;
    observation.range_sigma = camera_observation.range_sigma;
    observation.bearing_sigma = camera_observation.bearing_sigma;
    observations.push_back(observation);
  }
  pending_camera_observations_.clear();

  tracker_.Process(observations, rclcpp::Time(msg->header.stamp).seconds());

  RunCycle(msg->header);

  const auto elapsed = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
    SteadyClock::now() - start);
  last_latency_ms_ = elapsed.count();

  if (last_latency_ms_ > 18.0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "fusion cycle %.2f ms exceeds the 18 ms budget", last_latency_ms_);
  }
}

void SensorFusionNode::RunCycle(const std_msgs::msg::Header & header)
{
  sil_interfaces::msg::ObstacleArray output;
  output.header = header;
  output.header.frame_id = header.frame_id.empty() ? "base_link" : header.frame_id;
  output.cycle_id = ++cycle_count_;
  output.latency_ms = last_latency_ms_;

  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;

  for (const auto & track : tracker_.ConfirmedTracks()) {
    const StateVector & state = track.filter.state();

    sil_interfaces::msg::Obstacle obstacle;
    obstacle.header = output.header;
    obstacle.track_id = track.id;
    obstacle.position.x = state(0);
    obstacle.position.y = state(1);
    obstacle.position.z = 0.0;
    obstacle.velocity.x = state(2);
    obstacle.velocity.y = state(3);
    obstacle.velocity.z = 0.0;

    const CovarianceMatrix & p = track.filter.covariance();
    obstacle.position_covariance[0] = p(0, 0);
    obstacle.position_covariance[1] = p(0, 1);
    obstacle.position_covariance[2] = p(1, 0);
    obstacle.position_covariance[3] = p(1, 1);

    obstacle.closing_speed = track.filter.ClosingSpeed();
    obstacle.range = track.filter.Range();
    obstacle.mahalanobis_distance = track.mahalanobis;
    obstacle.hits = track.hits;
    obstacle.multimodal = track.seen_by_lidar && track.seen_by_camera;

    output.obstacles.push_back(obstacle);

    if (publish_markers_ && marker_pub_) {
      visualization_msgs::msg::Marker marker;
      marker.header = output.header;
      marker.ns = "sil_obstacles";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position = obstacle.position;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 4.5;
      marker.scale.y = 2.0;
      marker.scale.z = 1.6;
      marker.color.a = 0.6F;
      ClosingSpeedColour(obstacle.closing_speed, marker.color.r, marker.color.g,
        marker.color.b);
      marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      markers.markers.push_back(marker);
    }
  }

  obstacle_pub_->publish(output);
  if (publish_markers_ && marker_pub_) {
    marker_pub_->publish(markers);
  }
}

}  // namespace sil_sensor_fusion
