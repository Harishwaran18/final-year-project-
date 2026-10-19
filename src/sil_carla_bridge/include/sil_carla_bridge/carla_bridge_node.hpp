// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_CARLA_BRIDGE__CARLA_BRIDGE_NODE_HPP_
#define SIL_CARLA_BRIDGE__CARLA_BRIDGE_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sil_carla_bridge/frame_transform.hpp"
#include "sil_carla_bridge/lidar_synthesizer.hpp"
#include "sil_interfaces/msg/carla_actor_array.hpp"
#include "sil_interfaces/msg/detection2_d_array.hpp"
#include "std_msgs/msg/float32.hpp"

namespace sil_carla_bridge
{

/// Ingests CARLA ground truth and republishes it as ROS 2 sensor messages.
///
/// Two operating modes share one node:
///
///  * **Relay mode** (default when CARLA is present): forwards the simulator's
///    own LiDAR and camera topics, remapped onto the fusion node's inputs.
///  * **Synthesis mode** (default when CARLA is absent, which is the CI case):
///    converts the actor list into a ray-cast point cloud and pixel-space
///    bounding boxes, so the whole SIL pipeline runs without a simulator.
///
/// The synthetic path is deterministic, which is what makes the SIL tests
/// reproducible.
class CarlaBridgeNode : public rclcpp::Node
{
public:
  explicit CarlaBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  /// Number of actor lists ingested since start.
  std::uint64_t actorMessageCount() const {return actor_message_count_;}

  /// Number of sweeps synthesised (0 in pure relay mode).
  std::uint64_t synthesizedSweepCount() const {return synthesized_sweep_count_;}

  /// Number of camera frames synthesised (0 in pure relay mode).
  std::uint64_t synthesizedFrameCount() const {return synthesized_frame_count_;}

  /// Runs one synthesis step on @p actors and publishes the results. Exposed so
  /// tests can drive the node without a CARLA process.
  void ProcessActors(const sil_interfaces::msg::CarlaActorArray & actors);

private:
  void DeclareParameters();
  void LoadParameters();

  void OnActors(sil_interfaces::msg::CarlaActorArray::ConstSharedPtr msg);

  /// Converts the actor array into oriented boxes in the ego frame.
  std::vector<LidarSynthesizer::Box> BuildBoxes(
    const sil_interfaces::msg::CarlaActorArray & actors,
    const FrameTransform & transform) const;

  /// Converts the actor array into pixel-space bounding boxes.
  sil_interfaces::msg::Detection2DArray BuildDetections(
    const sil_interfaces::msg::CarlaActorArray & actors,
    const FrameTransform & transform) const;

  /// Publishes the ego speed derived from the CARLA ego velocity.
  void PublishEgoSpeed(const sil_interfaces::msg::CarlaActorArray & actors,
    const FrameTransform & transform);

  LidarSynthesizer synthesizer_;

  rclcpp::Subscription<sil_interfaces::msg::CarlaActorArray>::SharedPtr actor_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sil_interfaces::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;

  std::uint64_t actor_message_count_{0};
  std::uint64_t synthesized_sweep_count_{0};
  std::uint64_t synthesized_frame_count_{0};

  // Parameters.
  bool synthesis_mode_{true};
  std::string output_frame_id_{"base_link"};
  std::uint32_t lidar_channels_{32};
  std::uint32_t lidar_azimuth_samples_{1024};
  double lidar_max_range_{80.0};
  double lidar_range_noise_std_{0.02};
  double lidar_mount_height_{1.8};
  double camera_fov_horizontal_{1.5708};
  std::uint32_t camera_image_width_{1280};
  std::uint32_t camera_image_height_{720};
  double camera_fx_{1100.0};
  double camera_fy_{1100.0};
  double min_actor_range_{1.5};
  bool publish_ego_speed_{true};
};

}  // namespace sil_carla_bridge

#endif  // SIL_CARLA_BRIDGE__CARLA_BRIDGE_NODE_HPP_
