// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_CARLA_BRIDGE__SCENARIO_PUBLISHER_HPP_
#define SIL_CARLA_BRIDGE__SCENARIO_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "sil_interfaces/msg/carla_actor.hpp"
#include "sil_interfaces/msg/carla_actor_array.hpp"

namespace sil_carla_bridge
{

/// Publishes a scripted CARLA actor stream, standing in for the simulator.
///
/// The bridge synthesises sensors from an actor list; this node is what feeds it
/// when no CARLA process is attached, which is how the SIL stack is exercised in
/// CI. Scenarios are deterministic functions of the elapsed time, so a replay
/// always produces the same actor positions.
class ScenarioPublisher : public rclcpp::Node
{
public:
  explicit ScenarioPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  /// Actor list for @p elapsed_seconds into the scenario.
  sil_interfaces::msg::CarlaActorArray BuildActors(double elapsed_seconds) const;

  /// Number of actor arrays published so far.
  std::uint64_t publishCount() const {return publish_count_;}

  /// Runs one publish step; exposed so tests need no timer.
  void PublishStep();

private:
  /// Builds one actor with the given pose and class.
  static sil_interfaces::msg::CarlaActor MakeActor(
    std::uint32_t id, const std::string & class_label, double x, double y,
    double z, double yaw_deg, double half_length, double half_width,
    double half_height);

  void DeclareParameters();
  void LoadParameters();
  void OnTimer();

  rclcpp::Publisher<sil_interfaces::msg::CarlaActorArray>::SharedPtr actor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string scenario_{"lead_vehicle_braking"};
  double rate_hz_{20.0};
  std::uint64_t publish_count_{0};
  rclcpp::Time start_time_;
};

}  // namespace sil_carla_bridge

#endif  // SIL_CARLA_BRIDGE__SCENARIO_PUBLISHER_HPP_
