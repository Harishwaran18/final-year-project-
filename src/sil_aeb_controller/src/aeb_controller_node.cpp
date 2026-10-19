// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_aeb_controller/aeb_controller_node.hpp"

#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

namespace sil_aeb_controller
{
using namespace std::chrono_literals;

AebControllerNode::AebControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("sil_aeb_controller", options)
{
  DeclareParameters();
  LoadParameters();

  fsm_ = AebFsm(config_);
  last_obstacle_time_ = now();

  obstacle_sub_ = create_subscription<sil_interfaces::msg::ObstacleArray>(
    "fusion/obstacles", rclcpp::QoS(10),
    std::bind(&AebControllerNode::OnObstacles, this, std::placeholders::_1));

  speed_sub_ = create_subscription<std_msgs::msg::Float32>(
    "vehicle/ego_speed", rclcpp::QoS(10),
    std::bind(&AebControllerNode::OnEgoSpeed, this, std::placeholders::_1));

  alert_pub_ = create_publisher<sil_interfaces::msg::SafetyAlert>("aeb/alert", rclcpp::QoS(10));

  set_state_srv_ = create_service<sil_interfaces::srv::SetAebState>(
    "aeb/set_state",
    std::bind(&AebControllerNode::HandleSetState, this, std::placeholders::_1,
      std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&AebControllerNode::OnTimer, this));

  RCLCPP_INFO(get_logger(),
    "sil_aeb_controller ready: %.1f Hz, TTC thresholds [%.2f, %.2f, %.2f] s",
    loop_rate_hz_, config_.warning_ttc, config_.partial_braking_ttc,
    config_.full_braking_ttc);
}

void AebControllerNode::DeclareParameters()
{
  declare_parameter("loop_rate_hz", loop_rate_hz_);
  declare_parameter("warning_ttc", config_.warning_ttc);
  declare_parameter("partial_braking_ttc", config_.partial_braking_ttc);
  declare_parameter("full_braking_ttc", config_.full_braking_ttc);
  declare_parameter("release_ttc_margin", config_.release_ttc_margin);
  declare_parameter("min_state_duration", config_.min_state_duration);
  declare_parameter("min_closing_speed", config_.min_closing_speed);
  declare_parameter("lane_half_width", config_.lane_half_width);
  declare_parameter("max_detection_range", config_.max_detection_range);
  declare_parameter("emergency_distance", config_.emergency_distance);
  declare_parameter("standstill_gap", config_.standstill_gap);
  declare_parameter("reaction_time", config_.reaction_time);
  declare_parameter("partial_deceleration", config_.partial_deceleration);
  declare_parameter("full_deceleration", config_.full_deceleration);
  declare_parameter("max_deceleration", config_.max_deceleration);
  declare_parameter("obstacle_timeout", obstacle_timeout_);
}

void AebControllerNode::LoadParameters()
{
  loop_rate_hz_ = get_parameter("loop_rate_hz").as_double();
  config_.warning_ttc = get_parameter("warning_ttc").as_double();
  config_.partial_braking_ttc = get_parameter("partial_braking_ttc").as_double();
  config_.full_braking_ttc = get_parameter("full_braking_ttc").as_double();
  config_.release_ttc_margin = get_parameter("release_ttc_margin").as_double();
  config_.min_state_duration = get_parameter("min_state_duration").as_double();
  config_.min_closing_speed = get_parameter("min_closing_speed").as_double();
  config_.lane_half_width = get_parameter("lane_half_width").as_double();
  config_.max_detection_range = get_parameter("max_detection_range").as_double();
  config_.emergency_distance = get_parameter("emergency_distance").as_double();
  config_.standstill_gap = get_parameter("standstill_gap").as_double();
  config_.reaction_time = get_parameter("reaction_time").as_double();
  config_.partial_deceleration = get_parameter("partial_deceleration").as_double();
  config_.full_deceleration = get_parameter("full_deceleration").as_double();
  config_.max_deceleration = get_parameter("max_deceleration").as_double();
  obstacle_timeout_ = get_parameter("obstacle_timeout").as_double();
}

void AebControllerNode::OnObstacles(
  sil_interfaces::msg::ObstacleArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  input_.obstacles.clear();
  input_.obstacles.reserve(msg->obstacles.size());

  for (const auto & obstacle : msg->obstacles) {
    AebObstacle converted;
    converted.track_id = obstacle.track_id;
    converted.x = obstacle.position.x;
    converted.y = obstacle.position.y;
    converted.vx = obstacle.velocity.x;
    converted.vy = obstacle.velocity.y;
    converted.closing_speed = obstacle.closing_speed;
    converted.range = obstacle.range;
    input_.obstacles.push_back(converted);
  }
  last_obstacle_time_ = now();
}

void AebControllerNode::OnEgoSpeed(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ego_speed_ = static_cast<double>(msg->data);
}

void AebControllerNode::OnTimer()
{
  AebInput input_copy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    input_.timestamp = now().seconds();
    input_.ego_speed = ego_speed_;

    // Stale tracks must not keep the brakes applied; clear them so the FSM can
    // fall back to NORMAL.
    if ((now() - last_obstacle_time_).seconds() > obstacle_timeout_) {
      input_.obstacles.clear();
    }
    input_copy = input_;
  }

  const AebOutput output = fsm_.Update(input_copy);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_output_ = output;
    ++cycle_count_;
  }

  PublishAlert(output);

  if (output.state_changed) {
    RCLCPP_WARN(get_logger(),
      "AEB state -> %s (track=%u ttc=%.2f s distance=%.2f m decel=%.2f m/s^2)",
      ToString(output.state), output.target_track_id, output.ttc, output.distance,
      output.requested_deceleration);
  } else {
    RCLCPP_DEBUG(get_logger(), "AEB state %s ttc=%.2f s", ToString(output.state),
      output.ttc);
  }
}

void AebControllerNode::PublishAlert(const AebOutput & output)
{
  sil_interfaces::msg::SafetyAlert alert;
  alert.header.stamp = now();
  alert.header.frame_id = "base_link";
  alert.state = static_cast<std::uint8_t>(output.state);
  alert.target_track_id = output.target_track_id;
  alert.ttc = std::isfinite(output.ttc) ? output.ttc : 0.0;
  alert.distance = output.distance;
  alert.closing_speed = output.closing_speed;
  alert.requested_deceleration = output.requested_deceleration;
  alert.brake_pressure = output.brake_pressure;
  alert.state_changed = output.state_changed;
  alert_pub_->publish(alert);
}

void AebControllerNode::HandleSetState(
  const std::shared_ptr<sil_interfaces::srv::SetAebState::Request> request,
  std::shared_ptr<sil_interfaces::srv::SetAebState::Response> response)
{
  if (request->state > static_cast<std::uint8_t>(AebState::kFullEmergencyBrake)) {
    response->success = false;
    response->active_state = static_cast<std::uint8_t>(fsm_.state());
    response->message = "state out of range; expected 0..3";
    return;
  }

  if (request->state == static_cast<std::uint8_t>(AebState::kNormal)) {
    fsm_.ReleaseOverride();
  } else {
    fsm_.Override(static_cast<AebState>(request->state));
  }

  response->success = true;
  response->active_state = static_cast<std::uint8_t>(fsm_.state());
  response->message = request->latch ?
    "state latched" : "state override applied";
}

std::uint64_t AebControllerNode::cycleCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return cycle_count_;
}

AebOutput AebControllerNode::lastOutput() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_output_;
}

}  // namespace sil_aeb_controller
