// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_CAN_GATEWAY__CAN_GATEWAY_NODE_HPP_
#define SIL_CAN_GATEWAY__CAN_GATEWAY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "sil_can_gateway/can_frame_codec.hpp"
#include "sil_can_gateway/socketcan_transport.hpp"
#include "sil_interfaces/msg/safety_alert.hpp"
#include "std_msgs/msg/float32.hpp"

namespace sil_can_gateway
{

/// ROS 2 node that bridges the AEB stack onto a CAN bus.
///
/// Subscribes to @c aeb/alert and @c vehicle/ego_speed, then writes two classic
/// 8-byte frames per cycle to a SocketCAN interface (normally @c vcan0):
/// the brake command (0x1A0) and the target speed (0x1A1).
///
/// The node degrades gracefully: when the interface cannot be opened it keeps
/// running in dry-run mode and reports the error once, so the rest of the SIL
/// stack (and CI) is unaffected by a missing vcan interface.
class CanGatewayNode : public rclcpp::Node
{
public:
  explicit CanGatewayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  /// True when a CAN interface was successfully opened.
  bool isConnected() const;

  /// Number of transmit cycles completed.
  std::uint64_t transmitCount() const;

  /// Most recent frame written to the bus.
  CanFrame lastFrame() const;

  /// Attempts to open the configured interface; exposed for tests.
  bool Connect();

private:
  void DeclareParameters();
  void LoadParameters();

  void OnAlert(sil_interfaces::msg::SafetyAlert::ConstSharedPtr msg);
  void OnEgoSpeed(std_msgs::msg::Float32::ConstSharedPtr msg);
  void OnTimer();

  SocketCanTransport transport_;
  mutable std::mutex mutex_;
  BrakeCommand pending_command_;
  TargetSpeedCommand pending_speed_;
  bool has_alert_{false};
  std::uint8_t counter_{0};
  std::uint64_t transmit_count_{0};
  CanFrame last_frame_;
  bool connection_reported_{false};

  rclcpp::Subscription<sil_interfaces::msg::SafetyAlert>::SharedPtr alert_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters.
  std::string interface_name_{"vcan0"};
  double transmit_rate_hz_{50.0};
  bool loopback_{true};
  bool dry_run_{false};
  double alert_timeout_{0.5};
  double target_speed_{0.0};
  double ego_speed_{0.0};
  rclcpp::Time last_alert_time_;
};

}  // namespace sil_can_gateway

#endif  // SIL_CAN_GATEWAY__CAN_GATEWAY_NODE_HPP_
