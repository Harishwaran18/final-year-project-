// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_AEB_CONTROLLER__TTC_CALCULATOR_HPP_
#define SIL_AEB_CONTROLLER__TTC_CALCULATOR_HPP_

#include <cmath>
#include <cstdint>

namespace sil_aeb_controller
{

/// Time-to-collision and braking-distance maths for the AEB FSM.
///
/// All functions are pure and header-only so they can be unit tested with
/// GoogleTest and reused by SIL scenario scripts without linking ROS.
namespace ttc
{

/// Value returned when no collision is predicted (no closing motion, or the
/// geometry cannot produce a positive closing speed).
constexpr double kNoCollision = 1e9;

/// Computes time-to-collision [s] for a head-on approach.
///
/// TTC = distance / relative_velocity, with the relative velocity being the
/// closing speed between ego and target. A non-positive closing speed means the
/// gap is not shrinking, so @ref kNoCollision is returned.
///
/// @param distance        longitudinal gap to the target [m]
/// @param closing_speed   magnitude of the approach rate [m/s], >= 0
/// @return TTC in seconds, clamped to >= 0, or @ref kNoCollision
inline double TimeToCollision(double distance, double closing_speed)
{
  if (distance <= 0.0) {
    return 0.0;  // Already in contact: TTC is zero by definition.
  }
  if (closing_speed <= 0.0) {
    return kNoCollision;
  }
  return distance / closing_speed;
}

/// Minimum safe gap that must remain after braking [m].
///
/// @param ego_speed       current ego speed [m/s]
/// @param reaction_time   actuator + perception delay [s]
/// @param standstill_gap  bumper-to-bumper gap once stopped [m]
inline double SafetyMargin(
  double ego_speed, double reaction_time, double standstill_gap)
{
  return standstill_gap + ego_speed * reaction_time;
}

/// Distance travelled while decelerating from @p ego_speed to @p target_speed
/// at a constant @p deceleration [m].
///
/// Uses the kinematic relation d = (v0^2 - v1^2) / (2a). Returns 0 when the
/// target speed is not below the ego speed.
inline double BrakingDistance(
  double ego_speed, double target_speed, double deceleration)
{
  if (deceleration <= 0.0 || ego_speed <= target_speed) {
    return 0.0;
  }
  return (ego_speed * ego_speed - target_speed * target_speed) / (2.0 * deceleration);
}

/// Deceleration required to stop before @p distance [m/s^2].
///
/// Returns 0 when @p distance is non-positive (already overlapping).
inline double RequiredDeceleration(
  double ego_speed, double distance, double standstill_gap)
{
  const double available = distance - standstill_gap;
  if (available <= 0.0) {
    return 0.0;
  }
  return (ego_speed * ego_speed) / (2.0 * available);
}

/// Time available before impact under constant deceleration [s].
///
/// Solves d = v t - 0.5 a t^2 for the smaller positive root, i.e. the first
/// moment the gap closes. Returns @ref kNoCollision when the gap is never
/// closed: the ego is stationary, or @p deceleration is large enough to stop
/// the vehicle before it reaches @p distance.
inline double TimeToImpact(
  double distance, double ego_speed, double deceleration)
{
  if (distance <= 0.0) {
    return 0.0;
  }
  if (ego_speed <= 0.0) {
    return kNoCollision;
  }
  if (deceleration <= 0.0) {
    return distance / ego_speed;
  }

  // The braking distance at this deceleration must be short enough that the
  // vehicle is still moving when it reaches the obstacle; otherwise it stops
  // short and never arrives.
  if (BrakingDistance(ego_speed, 0.0, deceleration) <= distance) {
    return kNoCollision;
  }

  const double a = 0.5 * deceleration;
  const double b = -ego_speed;
  const double c = distance;
  const double discriminant = b * b - 4.0 * a * c;
  if (discriminant < 0.0) {
    return kNoCollision;
  }
  const double sqrt_disc = std::sqrt(discriminant);
  const double t1 = (-b - sqrt_disc) / (2.0 * a);
  const double t2 = (-b + sqrt_disc) / (2.0 * a);
  const double t = (t1 > 0.0 && t2 > 0.0) ? (t1 < t2 ? t1 : t2) : (t1 > t2 ? t1 : t2);
  return t > 0.0 ? t : 0.0;
}

}  // namespace ttc

}  // namespace sil_aeb_controller

#endif  // SIL_AEB_CONTROLLER__TTC_CALCULATOR_HPP_
