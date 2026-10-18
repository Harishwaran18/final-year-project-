// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_carla_bridge/carla_bridge_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "sil_carla_bridge/frame_transform.hpp"

namespace sil_carla_bridge
{
namespace
{
/// Half extents used when CARLA reports a degenerate bounding box.
constexpr double kDefaultHalfExtent = 0.9;
constexpr double kDefaultHalfHeight = 0.75;

/// Maps a CARLA blueprint class onto the labels the fusion front end expects.
std::string NormalizeClassLabel(const std::string & class_label)
{
  if (class_label.find("walker") != std::string::npos) {
    return "pedestrian";
  }
  if (class_label.find("truck") != std::string::npos) {
    return "truck";
  }
  if (class_label.find("bus") != std::string::npos) {
    return "bus";
  }
  if (class_label.find("bike") != std::string::npos ||
    class_label.find("motorcycle") != std::string::npos)
  {
    return "motorcycle";
  }
  if (class_label.find("vehicle") != std::string::npos ||
    class_label.find("car") != std::string::npos)
  {
    return "car";
  }
  if (class_label.find("traffic_light") != std::string::npos) {
    return "traffic_light";
  }
  if (class_label.find("traffic_sign") != std::string::npos) {
    return "traffic_sign";
  }
  return "unknown";
}
}  // namespace

CarlaBridgeNode::CarlaBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("sil_carla_bridge", options)
{
  DeclareParameters();
  LoadParameters();

  LidarSynthesizerConfig synthesizer_config;
  synthesizer_config.channels = lidar_channels_;
  synthesizer_config.azimuth_samples = lidar_azimuth_samples_;
  synthesizer_config.max_range = lidar_max_range_;
  synthesizer_config.range_noise_std = lidar_range_noise_std_;
  synthesizer_ = LidarSynthesizer(synthesizer_config);

  actor_sub_ = create_subscription<sil_interfaces::msg::CarlaActorArray>(
    "carla/actors", rclcpp::QoS(10),
    std::bind(&CarlaBridgeNode::OnActors, this, std::placeholders::_1));

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "lidar/points", rclcpp::SensorDataQoS());
  detection_pub_ = create_publisher<sil_interfaces::msg::Detection2DArray>(
    "camera/detections", rclcpp::SensorDataQoS());
  speed_pub_ = create_publisher<std_msgs::msg::Float32>("vehicle/ego_speed", rclcpp::QoS(10));

  RCLCPP_INFO(get_logger(),
    "sil_carla_bridge ready: mode=%s, lidar=%ux%u @ %.1f m",
    synthesis_mode_ ? "synthesis" : "relay", lidar_channels_, lidar_azimuth_samples_,
    lidar_max_range_);
}

void CarlaBridgeNode::DeclareParameters()
{
  declare_parameter("synthesis_mode", synthesis_mode_);
  declare_parameter("output_frame_id", output_frame_id_);
  declare_parameter("lidar_channels", static_cast<int>(lidar_channels_));
  declare_parameter("lidar_azimuth_samples", static_cast<int>(lidar_azimuth_samples_));
  declare_parameter("lidar_max_range", lidar_max_range_);
  declare_parameter("lidar_range_noise_std", lidar_range_noise_std_);
  declare_parameter("lidar_mount_height", lidar_mount_height_);
  declare_parameter("camera_fov_horizontal", camera_fov_horizontal_);
  declare_parameter("camera_image_width", static_cast<int>(camera_image_width_));
  declare_parameter("camera_image_height", static_cast<int>(camera_image_height_));
  declare_parameter("camera_fx", camera_fx_);
  declare_parameter("camera_fy", camera_fy_);
  declare_parameter("min_actor_range", min_actor_range_);
  declare_parameter("publish_ego_speed", publish_ego_speed_);
}

void CarlaBridgeNode::LoadParameters()
{
  synthesis_mode_ = get_parameter("synthesis_mode").as_bool();
  output_frame_id_ = get_parameter("output_frame_id").as_string();
  lidar_channels_ = static_cast<std::uint32_t>(get_parameter("lidar_channels").as_int());
  lidar_azimuth_samples_ =
    static_cast<std::uint32_t>(get_parameter("lidar_azimuth_samples").as_int());
  lidar_max_range_ = get_parameter("lidar_max_range").as_double();
  lidar_range_noise_std_ = get_parameter("lidar_range_noise_std").as_double();
  lidar_mount_height_ = get_parameter("lidar_mount_height").as_double();
  camera_fov_horizontal_ = get_parameter("camera_fov_horizontal").as_double();
  camera_image_width_ = static_cast<std::uint32_t>(get_parameter("camera_image_width").as_int());
  camera_image_height_ = static_cast<std::uint32_t>(get_parameter("camera_image_height").as_int());
  camera_fx_ = get_parameter("camera_fx").as_double();
  camera_fy_ = get_parameter("camera_fy").as_double();
  min_actor_range_ = get_parameter("min_actor_range").as_double();
  publish_ego_speed_ = get_parameter("publish_ego_speed").as_bool();
}

void CarlaBridgeNode::OnActors(
  sil_interfaces::msg::CarlaActorArray::ConstSharedPtr msg)
{
  ProcessActors(*msg);
}

std::vector<LidarSynthesizer::Box> CarlaBridgeNode::BuildBoxes(
  const sil_interfaces::msg::CarlaActorArray & actors,
  const FrameTransform & transform) const
{
  std::vector<LidarSynthesizer::Box> boxes;
  boxes.reserve(actors.actors.size());

  for (const auto & actor : actors.actors) {
    const Eigen::Vector3d world_position(
      actor.position.x, actor.position.y, actor.position.z);
    const Eigen::Vector3d world_center(
      world_position.x(), world_position.y(), world_position.z() - lidar_mount_height_);

    const double yaw = DegreesToRadians(actor.rotation.z);

    Eigen::Vector3d half_extents(
      actor.extent.x > 0.0 ? actor.extent.x : kDefaultHalfExtent,
      actor.extent.y > 0.0 ? actor.extent.y : kDefaultHalfExtent,
      actor.extent.z > 0.0 ? actor.extent.z : kDefaultHalfHeight);

    const std::string label = NormalizeClassLabel(actor.class_label);
    double intensity = 0.4;
    if (label == "pedestrian") {
      intensity = 0.2;
    } else if (label == "car" || label == "truck" || label == "bus") {
      intensity = 0.8;
    }

    LidarSynthesizer::Box box = TransformBoxToEgo(
      transform, world_center, half_extents, yaw, intensity, actor.actor_id, label);

    if (box.center.head<2>().norm() < min_actor_range_) {
      continue;  // Actor is inside the ego footprint.
    }
    boxes.push_back(box);
  }
  return boxes;
}

sil_interfaces::msg::Detection2DArray CarlaBridgeNode::BuildDetections(
  const sil_interfaces::msg::CarlaActorArray & actors,
  const FrameTransform & transform) const
{
  sil_interfaces::msg::Detection2DArray detections;
  detections.header.stamp = now();
  detections.header.frame_id = output_frame_id_;
  detections.fx = camera_fx_;
  detections.fy = camera_fy_;
  detections.cx = static_cast<double>(camera_image_width_) / 2.0;
  detections.cy = static_cast<double>(camera_image_height_) / 2.0;
  detections.image_width = camera_image_width_;
  detections.image_height = camera_image_height_;

  std::uint32_t detection_id = 0;
  for (const auto & actor : actors.actors) {
    const Eigen::Vector3d world_position(
      actor.position.x, actor.position.y, actor.position.z);
    const Eigen::Vector3d ego_position = transform.WorldToEgo(world_position);

    if (ego_position.x() <= min_actor_range_) {
      continue;  // Behind the camera or inside the ego body.
    }

    // Project the actor centre through the pinhole model.
    const double u = camera_fx_ * (ego_position.y() / ego_position.x()) + detections.cx;
    const double v = -camera_fy_ * (ego_position.z() / ego_position.x()) + detections.cy;

    if (u < 0.0 || u >= static_cast<double>(camera_image_width_) ||
      v < 0.0 || v >= static_cast<double>(camera_image_height_))
    {
      continue;  // Outside the image plane.
    }

    // Apparent size from the pinhole model, clamped to the image bounds.
    const double half_width_px = std::max(4.0, camera_fx_ *
      (actor.extent.x > 0.0 ? actor.extent.x : kDefaultHalfExtent) / ego_position.x());
    const double half_height_px = std::max(4.0, camera_fy_ *
      (actor.extent.z > 0.0 ? actor.extent.z : kDefaultHalfHeight) / ego_position.x());

    const double x_min = std::max(0.0, u - half_width_px);
    const double y_min = std::max(0.0, v - half_height_px);
    const double x_max = std::min(static_cast<double>(camera_image_width_), u + half_width_px);
    const double y_max = std::min(static_cast<double>(camera_image_height_), v + half_height_px);
    if (x_max <= x_min || y_max <= y_min) {
      continue;
    }

    sil_interfaces::msg::Detection2D detection;
    detection.header = detections.header;
    detection.detection_id = detection_id++;
    detection.class_label = NormalizeClassLabel(actor.class_label);
    detection.x_offset = static_cast<std::uint32_t>(x_min);
    detection.y_offset = static_cast<std::uint32_t>(y_min);
    detection.width = static_cast<std::uint32_t>(x_max - x_min);
    detection.height = static_cast<std::uint32_t>(y_max - y_min);
    detection.score = 0.9F;
    detections.detections.push_back(detection);
  }
  return detections;
}

void CarlaBridgeNode::PublishEgoSpeed(
  const sil_interfaces::msg::CarlaActorArray & actors,
  const FrameTransform & transform)
{
  if (!publish_ego_speed_) {
    return;
  }
  const Eigen::Vector3d world_velocity(
    actors.ego_velocity.x, actors.ego_velocity.y, actors.ego_velocity.z);
  const Eigen::Vector3d ego_velocity = transform.RotateVectorToEgo(world_velocity);

  std_msgs::msg::Float32 speed;
  // Longitudinal speed only; a lateral component is not AEB-relevant.
  speed.data = static_cast<float>(std::max(0.0, ego_velocity.x()));
  speed_pub_->publish(speed);
}

void CarlaBridgeNode::ProcessActors(const sil_interfaces::msg::CarlaActorArray & actors)
{
  Pose ego_pose;
  ego_pose.position = Eigen::Vector3d(
    actors.ego_position.x, actors.ego_position.y, actors.ego_position.z);
  ego_pose.rpy = Eigen::Vector3d(
    DegreesToRadians(actors.ego_rotation.x),
    DegreesToRadians(actors.ego_rotation.y),
    DegreesToRadians(actors.ego_rotation.z));
  const FrameTransform transform(ego_pose);

  ++actor_message_count_;
  PublishEgoSpeed(actors, transform);

  if (!synthesis_mode_) {
    return;  // Relay mode: the simulator publishes the sensors itself.
  }

  const std::vector<LidarSynthesizer::Box> boxes = BuildBoxes(actors, transform);

  pcl::PointCloud<pcl::PointXYZI> cloud = synthesizer_.Synthesize(boxes);

  // Mount the sensor at the configured height above the ego origin.
  for (auto & point : cloud.points) {
    point.z += static_cast<float>(lidar_mount_height_);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = now();
  cloud_msg.header.frame_id = output_frame_id_;
  cloud_pub_->publish(cloud_msg);
  ++synthesized_sweep_count_;

  detection_pub_->publish(BuildDetections(actors, transform));
  ++synthesized_frame_count_;
}

}  // namespace sil_carla_bridge
