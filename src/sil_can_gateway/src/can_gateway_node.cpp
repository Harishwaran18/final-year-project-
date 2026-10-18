// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_can_gateway/can_gateway_node.hpp"

#include <chrono>
#include <utility>

namespace sil_can_gateway
{

CanGatewayNode::CanGatewayNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("sil_can_gateway", options)
{
  DeclareParameters();
  LoadParameters();

  last_alert_time_ = now();

  alert_sub_ = create_subscription<sil_interfaces::msg::SafetyAlert>(
    "aeb/alert", rclcpp::QoS(10),
    std::bind(&CanGatewayNode::OnAlert, this, std::placeholders::_1));

  speed_sub_ = create_subscription<std_msgs::msg::Float32>(
    "vehicle/ego_speed", rclcpp::QoS(10),
    std::bind(&CanGatewayNode::OnEgoSpeed, this, std::placeholders::_1));

  if (!dry_run_) {
    Connect();
  } else {
    RCLCPP_INFO(get_logger(), "sil_can_gateway started in dry-run mode; no frames are sent");
  }

  const auto period = std::chrono::duration<double>(1.0 / transmit_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CanGatewayNode::OnTimer, this));
}

void CanGatewayNode::DeclareParameters()
{
  declare_parameter("interface_name", interface_name_);
  declare_parameter("transmit_rate_hz", transmit_rate_hz_);
  declare_parameter("loopback", loopback_);
  declare_parameter("dry_run", dry_run_);
  declare_parameter("alert_timeout", alert_timeout_);
}

void CanGatewayNode::LoadParameters()
{
  interface_name_ = get_parameter("interface_name").as_string();
  transmit_rate_hz_ = get_parameter("transmit_rate_hz").as_double();
  loopback_ = get_parameter("loopback").as_bool();
  dry_run_ = get_parameter("dry_run").as_bool();
  alert_timeout_ = get_parameter("alert_timeout").as_double();
}

bool CanGatewayNode::Connect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (transport_.is_open()) {
    return true;
  }

  if (!transport_.Open(interface_name_, loopback_)) {
    RCLCPP_ERROR(get_logger(),
      "cannot open CAN interface '%s' (%s); running in dry-run mode",
      interface_name_.c_str(), transport_.last_error().c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "SocketCAN interface '%s' ready (loopback=%s)",
    interface_name_.c_str(), loopback_ ? "on" : "off");
  return true;
}

void CanGatewayNode::OnAlert(sil_interfaces::msg::SafetyAlert::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_command_.requested_deceleration = msg->requested_deceleration;
  pending_command_.brake_pressure = msg->brake_pressure;
  pending_command_.state = msg->state;
  pending_command_.target_track_id = static_cast<std::uint16_t>(msg->target_track_id);
  pending_command_.state_changed = msg->state_changed;
  pending_command_.valid = true;
  has_alert_ = true;
  last_alert_time_ = now();
}

void CanGatewayNode::OnEgoSpeed(std_msgs::msg::Float32::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ego_speed_ = static_cast<double>(msg->data);
}

void CanGatewayNode::OnTimer()
{
  BrakeCommand brake;
  TargetSpeedCommand speed;
  std::uint8_t counter = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // A stale or absent alert means the controller is gone: fall back to a
    // zero-deceleration, NORMAL command rather than latching the last brake
    // request, which would be the unsafe failure mode.
    const bool stale = !has_alert_ ||
      (now() - last_alert_time_).seconds() > alert_timeout_;
    if (stale) {
      pending_command_ = BrakeCommand{};
      pending_command_.state = 0;
    }

    brake = pending_command_;
    speed.target_speed = target_speed_;
    speed.ego_speed = ego_speed_;
    speed.deceleration = pending_command_.requested_deceleration;
    speed.brake_pressure = pending_command_.brake_pressure;
    counter = static_cast<std::uint8_t>(counter_++ & protocol::kCounterMask);
  }

  const CanFrame brake_frame = EncodeBrakeCommand(brake, counter);
  const CanFrame speed_frame = EncodeTargetSpeed(speed, counter);

  if (transport_.is_open()) {
    if (!transport_.Send(brake_frame)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "failed to send brake frame: %s", transport_.last_error().c_str());
    }
    if (!transport_.Send(speed_frame)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "failed to send target-speed frame: %s", transport_.last_error().c_str());
    }
  } else if (!connection_reported_ && !dry_run_) {
    connection_reported_ = true;
    RCLCPP_WARN(get_logger(),
      "CAN interface unavailable; frames are encoded but not transmitted");
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_frame_ = brake_frame;
    ++transmit_count_;
  }
}

bool CanGatewayNode::isConnected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return transport_.is_open();
}

std::uint64_t CanGatewayNode::transmitCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return transmit_count_;
}

CanFrame CanGatewayNode::lastFrame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_frame_;
}

}  // namespace sil_can_gateway
