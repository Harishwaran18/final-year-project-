// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_AEB_CONTROLLER__AEB_FSM_HPP_
#define SIL_AEB_CONTROLLER__AEB_FSM_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace sil_aeb_controller
{

/// AEB finite state machine states.
///
/// The numeric values are part of the published interface
/// (sil_interfaces/msg/SafetyAlert) and must not be reordered.
enum class AebState : std::uint8_t
{
  kNormal = 0,
  kForwardCollisionWarning = 1,
  kPartialBraking = 2,
  kFullEmergencyBrake = 3,
};

/// Human readable state name, used in logs and tests.
const char * ToString(AebState state);

/// A candidate obstacle as seen by the AEB controller.
struct AebObstacle
{
  std::uint32_t track_id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  /// Closing speed along the ego axis [m/s], positive == closing.
  double closing_speed{0.0};
  /// Euclidean range from the ego origin [m].
  double range{0.0};
};

/// Tunable thresholds and actuator limits for the AEB FSM.
struct AebConfig
{
  // --- Escalation thresholds on TTC [s]. -----------------------------------
  double warning_ttc{2.5};
  double partial_braking_ttc{1.6};
  double full_braking_ttc{0.8};

  // --- Release hysteresis. --------------------------------------------------
  /// TTC margin that must be exceeded before de-escalating [s]. Prevents
  /// state chatter when TTC hovers on a threshold.
  double release_ttc_margin{0.4};
  /// Minimum time a state must be held before de-escalation [s].
  /// Escalation is never delayed: AEB must fail safe, not fail slow.
  double min_state_duration{0.4};

  // --- Geometry gates. ------------------------------------------------------
  /// Closing speeds below this are treated as stationary traffic [m/s].
  double min_closing_speed{1.0};
  /// Lateral half-width of the ego lane [m]; obstacles outside are ignored.
  double lane_half_width{2.0};
  /// Maximum longitudinal range considered [m].
  double max_detection_range{80.0};
  /// Range below which a full emergency brake is forced regardless of TTC [m].
  double emergency_distance{4.0};
  /// Gap left between bumpers once stopped [m].
  double standstill_gap{2.0};

  // --- Actuator model. ------------------------------------------------------
  /// Perception + actuator delay used for the safety margin [s].
  double reaction_time{0.15};
  /// Nominal deceleration requested in PARTIAL_BRAKING [m/s^2].
  double partial_deceleration{3.5};
  /// Nominal deceleration requested in FULL_EMERGENCY_BRAKE [m/s^2].
  double full_deceleration{9.0};
  /// Physical limit of the brake system [m/s^2].
  double max_deceleration{9.5};
  /// Normalised brake pressure requested in PARTIAL_BRAKING.
  double partial_brake_pressure{0.35};
  /// Normalised brake pressure requested in FULL_EMERGENCY_BRAKE.
  double full_brake_pressure{1.0};
};

/// Input to one FSM cycle.
struct AebInput
{
  double timestamp{0.0};
  double ego_speed{0.0};
  /// Obstacles visible to the controller this cycle.
  std::vector<AebObstacle> obstacles;
};

/// Output of one FSM cycle.
struct AebOutput
{
  AebState state{AebState::kNormal};
  std::uint32_t target_track_id{0};
  double ttc{0.0};
  double distance{0.0};
  double closing_speed{0.0};
  double requested_deceleration{0.0};
  double brake_pressure{0.0};
  bool state_changed{false};
  /// True when the state was driven by an external override.
  bool overridden{false};
};

/// Selects the most threatening in-lane obstacle.
///
/// Threat ranking is by ascending TTC, with range as the tie breaker. Obstacles
/// outside the lane or beyond the detection range are not considered.
class TargetSelector
{
public:
  explicit TargetSelector(const AebConfig & config = AebConfig{});

  /// @return the index into @p obstacles of the selected target, or -1 when
  ///         there is no threatening target.
  int Select(const std::vector<AebObstacle> & obstacles, double ego_speed) const;

  /// TTC of @p obstacle given the ego speed, using closing speed when it is
  /// reported and falling back to the relative velocity otherwise [s].
  double ComputeTtc(const AebObstacle & obstacle, double ego_speed) const;

  const AebConfig & config() const {return config_;}

private:
  AebConfig config_;
};

/// Deterministic AEB finite state machine.
///
/// Escalation is immediate (safety critical). De-escalation is rate limited by
/// @ref AebConfig::min_state_duration and by a TTC release margin, which is what
/// keeps the state from oscillating around a threshold.
class AebFsm
{
public:
  explicit AebFsm(const AebConfig & config = AebConfig{});

  /// Runs one control cycle.
  AebOutput Update(const AebInput & input);

  /// Latches the FSM into @p state, bypassing the automatic transitions.
  /// Used by the SIL harness and HIL benches via SetAebState.
  void Override(AebState state);

  /// Clears a previously latched override.
  void ReleaseOverride();

  /// Forgets all history (state, timers, override); used between scenarios.
  void Reset();

  AebState state() const {return state_;}
  bool overridden() const {return override_active_;}
  const AebConfig & config() const {return config_;}

  /// TTC at which the given state escalates or releases, exposed for tests.
  double EscalationTtc(AebState state) const;
  double ReleaseTtc(AebState state) const;

private:
  /// Maps a TTC and distance to the state the controller wants to be in.
  AebState EvaluateDesiredState(double ttc, double distance) const;

  /// Builds the actuator command for @p state.
  void ApplyActuatorCommand(AebState state, double ego_speed, AebOutput & output) const;

  AebConfig config_;
  TargetSelector selector_;
  AebState state_{AebState::kNormal};
  double state_entry_time_{0.0};
  bool override_active_{false};
  AebState override_state_{AebState::kNormal};
  bool has_time_{false};
};

}  // namespace sil_aeb_controller

#endif  // SIL_AEB_CONTROLLER__AEB_FSM_HPP_
