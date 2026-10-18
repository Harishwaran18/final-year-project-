// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_AEB_CONTROLLER__AEB_CONTROLLER_NODE_HPP_
#define SIL_AEB_CONTROLLER__AEB_CONTROLLER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include "sil_aeb_controller/aeb_fsm.hpp"
#include "sil_interfaces/msg/obstacle_array.hpp"
#include "sil_interfaces/msg/safety_alert.hpp"
#include "sil_interfaces/srv/set_aeb_state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace sil_aeb_controller
{

/// ROS 2 wrapper around @ref AebFsm.
///
/// Consumes fused obstacle tracks and the ego speed, evaluates the AEB FSM at a
/// fixed rate, and publishes a @c SafetyAlert for the CAN gateway. A
/// @c SetAebState service lets the SIL harness force states.
class AebControllerNode : public rclcpp::Node
{
public:
  explicit AebControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  /// Number of FSM cycles executed; used by tests and diagnostics.
  std::uint64_t cycleCount() const;

  /// Most recent FSM output, copied under the state mutex.
  AebOutput lastOutput() const;

private:
  void DeclareParameters();
  void LoadParameters();

  void OnObstacles(sil_interfaces::msg::ObstacleArray::ConstSharedPtr msg);
  void OnEgoSpeed(std_msgs::msg::Float32::ConstSharedPtr msg);
  void OnTimer();

  void HandleSetState(
    const std::shared_ptr<sil_interfaces::srv::SetAebState::Request> request,
    std::shared_ptr<sil_interfaces::srv::SetAebState::Response> response);

  /// Publishes a @c SafetyAlert from an FSM output.
  void PublishAlert(const AebOutput & output);

  AebConfig config_;
  AebFsm fsm_;

  mutable std::mutex mutex_;
  AebInput input_;
  AebOutput last_output_;
  std::uint64_t cycle_count_{0};

  rclcpp::Subscription<sil_interfaces::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr speed_sub_;
  rclcpp::Publisher<sil_interfaces::msg::SafetyAlert>::SharedPtr alert_pub_;
  rclcpp::Service<sil_interfaces::srv::SetAebState>::SharedPtr set_state_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters.
  double loop_rate_hz_{50.0};
  double ego_speed_{0.0};
  double obstacle_timeout_{0.5};
  rclcpp::Time last_obstacle_time_;
};

}  // namespace sil_aeb_controller

#endif  // SIL_AEB_CONTROLLER__AEB_CONTROLLER_NODE_HPP_
