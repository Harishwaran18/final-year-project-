// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_carla_bridge/scenario_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace sil_carla_bridge
{
namespace
{
/// Ego starts here and travels forward at a constant 16.7 m/s (60 km/h).
constexpr double kEgoSpeed = 16.7;
constexpr double kEgoStartX = 0.0;

/// The lead vehicle starts 55 m ahead and decelerates from 12 m/s to a stop.
constexpr double kLeadStartGap = 55.0;
constexpr double kLeadInitialSpeed = 12.0;
constexpr double kLeadDeceleration = 4.0;

/// A pedestrian stepping out 30 m ahead, used by the crossing scenario.
constexpr double kPedestrianStartGap = 30.0;
constexpr double kPedestrianLateralSpeed = 1.4;
}  // namespace

ScenarioPublisher::ScenarioPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("carla_scenario_publisher", options)
{
  DeclareParameters();
  LoadParameters();

  start_time_ = now();

  actor_pub_ = create_publisher<sil_interfaces::msg::CarlaActorArray>(
    "carla/actors", rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ScenarioPublisher::OnTimer, this));

  RCLCPP_INFO(get_logger(), "carla_scenario_publisher ready: scenario='%s' at %.1f Hz",
    scenario_.c_str(), rate_hz_);
}

void ScenarioPublisher::DeclareParameters()
{
  declare_parameter("scenario", scenario_);
  declare_parameter("rate_hz", rate_hz_);
}

void ScenarioPublisher::LoadParameters()
{
  scenario_ = get_parameter("scenario").as_string();
  rate_hz_ = get_parameter("rate_hz").as_double();
}

sil_interfaces::msg::CarlaActor ScenarioPublisher::MakeActor(
  std::uint32_t id, const std::string & class_label, double x, double y, double z,
  double yaw_deg, double half_length, double half_width, double half_height)
{
  sil_interfaces::msg::CarlaActor actor;
  actor.actor_id = id;
  actor.class_label = class_label;
  actor.position.x = x;
  actor.position.y = y;
  actor.position.z = z;
  actor.rotation.z = yaw_deg;
  actor.extent.x = half_length;
  actor.extent.y = half_width;
  actor.extent.z = half_height;
  return actor;
}

sil_interfaces::msg::CarlaActorArray ScenarioPublisher::BuildActors(
  double elapsed_seconds) const
{
  sil_interfaces::msg::CarlaActorArray actors;
  actors.header.stamp = now();

  // Ego travels at a constant speed along +x in the CARLA frame.
  const double ego_x = kEgoStartX + kEgoSpeed * elapsed_seconds;
  actors.ego_position.x = ego_x;
  actors.ego_position.y = 0.0;
  actors.ego_position.z = 0.75;
  actors.ego_rotation.z = 0.0;
  actors.ego_velocity.x = kEgoSpeed;

  if (scenario_ == "pedestrian_crossing") {
    // The pedestrian walks across the ego path from the right-hand kerb.
    const double crossing_offset = kPedestrianLateralSpeed * elapsed_seconds;
    actors.actors.push_back(MakeActor(
      2, "walker.pedestrian.0001", kPedestrianStartGap, -2.5 + crossing_offset,
      0.9, 0.0, 0.3, 0.3, 0.9));
    return actors;
  }

  if (scenario_ == "static_obstacle") {
    // A stopped vehicle parked in the ego lane.
    actors.actors.push_back(MakeActor(
      3, "vehicle.tesla.model3", 45.0, 0.0, 0.75, 0.0, 2.4, 1.0, 0.75));
    return actors;
  }

  if (scenario_ == "lead_vehicle_constant") {
    // A lead vehicle holding a steady 12 m/s, so the gap closes slowly.
    const double lead_x = kEgoStartX + kLeadStartGap + kLeadInitialSpeed * elapsed_seconds;
    actors.actors.push_back(MakeActor(
      1, "vehicle.tesla.model3", lead_x, 0.0, 0.75, 0.0, 2.4, 1.0, 0.75));
    return actors;
  }

  // Default: "lead_vehicle_braking". The lead vehicle decelerates to a stop,
  // which is the canonical AEB trigger.
  const double stop_time = kLeadInitialSpeed / kLeadDeceleration;
  const double braking_time = std::min(elapsed_seconds, stop_time);
  const double lead_x = kEgoStartX + kLeadStartGap +
    kLeadInitialSpeed * braking_time - 0.5 * kLeadDeceleration * braking_time * braking_time;

  sil_interfaces::msg::CarlaActor lead = MakeActor(
    1, "vehicle.tesla.model3", lead_x, 0.0, 0.75, 0.0, 2.4, 1.0, 0.75);
  lead.velocity.x = std::max(0.0, kLeadInitialSpeed - kLeadDeceleration * elapsed_seconds);
  actors.actors.push_back(lead);

  // A second vehicle in the adjacent lane, to prove the lane gate works.
  actors.actors.push_back(MakeActor(
    4, "vehicle.audi.a2", kEgoStartX + 35.0, 4.0, 0.75, 0.0, 2.0, 0.9, 0.75));
  return actors;
}

void ScenarioPublisher::PublishStep()
{
  const double elapsed = (now() - start_time_).seconds();
  actor_pub_->publish(BuildActors(elapsed));
  ++publish_count_;
}

void ScenarioPublisher::OnTimer()
{
  PublishStep();
}

}  // namespace sil_carla_bridge
