// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_aeb_controller/aeb_fsm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sil_aeb_controller/ttc_calculator.hpp"

namespace sil_aeb_controller
{
namespace
{
/// Ego-axis half width used to decide whether an obstacle is in lane.
constexpr double kEpsilon = 1e-6;
/// Relative speed below which a target counts as stationary [m/s].
constexpr double kStationaryVelocityEpsilon = 0.5;
}  // namespace

const char * ToString(AebState state)
{
  switch (state) {
    case AebState::kNormal:
      return "NORMAL";
    case AebState::kForwardCollisionWarning:
      return "FORWARD_COLLISION_WARNING";
    case AebState::kPartialBraking:
      return "PARTIAL_BRAKING";
    case AebState::kFullEmergencyBrake:
      return "FULL_EMERGENCY_BRAKE";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// TargetSelector
// ---------------------------------------------------------------------------

TargetSelector::TargetSelector(const AebConfig & config)
: config_(config) {}

double TargetSelector::ComputeTtc(const AebObstacle & obstacle, double ego_speed) const
{
  const double range = obstacle.range > 0.0 ? obstacle.range :
    std::hypot(obstacle.x, obstacle.y);

  double closing = obstacle.closing_speed;
  if (closing <= 0.0 && range > kEpsilon) {
    // Fall back to the radial projection of the relative velocity. The obstacle
    // velocity is expressed in the ego frame, so a positive projection onto the
    // line of sight means the gap is shrinking.
    closing = -(obstacle.x * obstacle.vx + obstacle.y * obstacle.vy) / range;
  }

  // Only a target with no measurable relative motion is treated as stationary
  // traffic, in which case the ego speed is the worst-case approach rate. This
  // guard matters: without it a target driving away would also be assumed
  // stationary and would trigger a spurious emergency brake.
  if (closing <= 0.0 && ego_speed > 0.0 &&
    std::hypot(obstacle.vx, obstacle.vy) < kStationaryVelocityEpsilon)
  {
    closing = ego_speed;
  }

  return ttc::TimeToCollision(range, closing);
}

int TargetSelector::Select(
  const std::vector<AebObstacle> & obstacles, double ego_speed) const
{
  int best_index = -1;
  double best_ttc = std::numeric_limits<double>::max();
  double best_range = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const AebObstacle & obstacle = obstacles[i];

    // Longitudinal gate: only consider obstacles ahead of the ego bumper.
    if (obstacle.x <= 0.0) {
      continue;
    }
    // Lateral gate: obstacles outside the ego lane cannot be hit head-on.
    if (std::abs(obstacle.y) > config_.lane_half_width) {
      continue;
    }

    const double range = obstacle.range > 0.0 ? obstacle.range :
      std::hypot(obstacle.x, obstacle.y);
    if (range > config_.max_detection_range) {
      continue;
    }

    const double ttc_value = ComputeTtc(obstacle, ego_speed);
    if (ttc_value >= ttc::kNoCollision) {
      continue;  // Not closing: cannot be the AEB target.
    }

    if (ttc_value < best_ttc ||
      (std::abs(ttc_value - best_ttc) < kEpsilon && range < best_range))
    {
      best_ttc = ttc_value;
      best_range = range;
      best_index = static_cast<int>(i);
    }
  }

  return best_index;
}

// ---------------------------------------------------------------------------
// AebFsm
// ---------------------------------------------------------------------------

AebFsm::AebFsm(const AebConfig & config)
: config_(config), selector_(config) {}

void AebFsm::Reset()
{
  state_ = AebState::kNormal;
  state_entry_time_ = 0.0;
  override_active_ = false;
  override_state_ = AebState::kNormal;
  has_time_ = false;
}

void AebFsm::Override(AebState state)
{
  override_active_ = true;
  override_state_ = state;
  // Apply immediately rather than waiting for the next Update(): a caller that
  // forces a state (the HIL service, a test bench) reads state() straight back
  // and must not see the pre-override value.
  state_ = state;
}

void AebFsm::ReleaseOverride()
{
  override_active_ = false;
  // Drop the state-entry timestamp: min_state_duration exists to damp
  // automatic de-escalation, but an explicit release is an operator command
  // and must return to automatic control on the next cycle rather than waiting
  // out the hold time of the state the override had forced.
  has_time_ = false;
}

double AebFsm::EscalationTtc(AebState state) const
{
  switch (state) {
    case AebState::kNormal:
      return ttc::kNoCollision;
    case AebState::kForwardCollisionWarning:
      return config_.warning_ttc;
    case AebState::kPartialBraking:
      return config_.partial_braking_ttc;
    case AebState::kFullEmergencyBrake:
      return config_.full_braking_ttc;
  }
  return ttc::kNoCollision;
}

double AebFsm::ReleaseTtc(AebState state) const
{
  // De-escalation requires the TTC to clear the threshold by the margin.
  return EscalationTtc(state) + config_.release_ttc_margin;
}

AebState AebFsm::EvaluateDesiredState(double ttc_value, double distance) const
{
  // A near-field target forces the maximum intervention even if the closing
  // speed estimate is poor; this is the fail-safe path.
  if (distance > 0.0 && distance <= config_.emergency_distance) {
    return AebState::kFullEmergencyBrake;
  }
  if (ttc_value <= config_.full_braking_ttc) {
    return AebState::kFullEmergencyBrake;
  }
  if (ttc_value <= config_.partial_braking_ttc) {
    return AebState::kPartialBraking;
  }
  if (ttc_value <= config_.warning_ttc) {
    return AebState::kForwardCollisionWarning;
  }
  return AebState::kNormal;
}

void AebFsm::ApplyActuatorCommand(
  AebState state, double ego_speed, AebOutput & output) const
{
  switch (state) {
    case AebState::kNormal:
    case AebState::kForwardCollisionWarning:
      output.requested_deceleration = 0.0;
      output.brake_pressure = 0.0;
      return;
    case AebState::kPartialBraking:
    case AebState::kFullEmergencyBrake:
      break;
  }

  const double nominal = state == AebState::kFullEmergencyBrake ?
    config_.full_deceleration : config_.partial_deceleration;

  // Never ask for less than what the current geometry requires, and never
  // exceed the physical limit of the brake system.
  const double required = ttc::RequiredDeceleration(
    ego_speed, output.distance, config_.standstill_gap);
  const double deceleration = std::min(
    config_.max_deceleration, std::max(nominal, required));

  output.requested_deceleration = deceleration;
  output.brake_pressure = std::min(1.0, deceleration / config_.max_deceleration);

  // Keep the partial-braking request distinguishable from a full request so the
  // CAN gateway can encode the intended severity.
  if (state == AebState::kPartialBraking) {
    output.brake_pressure = std::min(output.brake_pressure, config_.partial_brake_pressure);
  }
}

AebOutput AebFsm::Update(const AebInput & input)
{
  AebOutput output;
  const AebState previous_state = state_;

  // --- 1. Resolve an external override first. ------------------------------
  if (override_active_) {
    state_ = override_state_;
    output.overridden = true;
  } else {
    // --- 2. Select the most threatening in-lane obstacle. ------------------
    const int index = selector_.Select(input.obstacles, input.ego_speed);
    if (index >= 0) {
      const AebObstacle & target = input.obstacles[static_cast<std::size_t>(index)];
      output.target_track_id = target.track_id;
      output.distance = target.range > 0.0 ? target.range :
        std::hypot(target.x, target.y);
      output.closing_speed = target.closing_speed;
      output.ttc = selector_.ComputeTtc(target, input.ego_speed);

      const AebState desired = EvaluateDesiredState(output.ttc, output.distance);

      if (static_cast<std::uint8_t>(desired) > static_cast<std::uint8_t>(state_)) {
        // Escalate immediately: never delay a safety intervention.
        state_ = desired;
      } else if (static_cast<std::uint8_t>(desired) < static_cast<std::uint8_t>(state_)) {
        // De-escalate only after the release margin and the hold time.
        const bool ttc_cleared = output.ttc > ReleaseTtc(state_);
        const bool held_long_enough = !has_time_ ||
          (input.timestamp - state_entry_time_) >= config_.min_state_duration;
        if (ttc_cleared && held_long_enough) {
          state_ = desired;
        }
      }
    } else {
      // No threatening target: fall back to NORMAL respecting the hold time.
      output.ttc = ttc::kNoCollision;
      output.distance = 0.0;
      output.closing_speed = 0.0;
      const bool held_long_enough = !has_time_ ||
        (input.timestamp - state_entry_time_) >= config_.min_state_duration;
      if (state_ != AebState::kNormal && held_long_enough) {
        state_ = AebState::kNormal;
      }
    }
  }

  // --- 3. Latch the entry time on every state change. ---------------------
  if (state_ != previous_state || !has_time_) {
    state_entry_time_ = input.timestamp;
    has_time_ = true;
  }

  output.state = state_;
  output.state_changed = state_ != previous_state;

  // --- 4. Translate the state into an actuator command. -------------------
  ApplyActuatorCommand(state_, input.ego_speed, output);

  if (output.overridden && state_ == AebState::kNormal) {
    output.requested_deceleration = 0.0;
    output.brake_pressure = 0.0;
  }
  return output;
}

}  // namespace sil_aeb_controller
