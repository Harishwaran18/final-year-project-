// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "sil_aeb_controller/aeb_fsm.hpp"

namespace sil_aeb_controller
{
namespace
{

AebObstacle MakeObstacle(std::uint32_t id, double x, double closing_speed)
{
  AebObstacle obstacle;
  obstacle.track_id = id;
  obstacle.x = x;
  obstacle.y = 0.0;
  obstacle.vx = -closing_speed;
  obstacle.vy = 0.0;
  obstacle.closing_speed = closing_speed;
  obstacle.range = std::abs(x);
  return obstacle;
}

AebInput MakeInput(double timestamp, double ego_speed, std::vector<AebObstacle> obstacles)
{
  AebInput input;
  input.timestamp = timestamp;
  input.ego_speed = ego_speed;
  input.obstacles = std::move(obstacles);
  return input;
}

/// Builds an input whose TTC is exactly @p ttc by placing a target at a range
/// derived from the configured closing speed.
AebInput InputWithTtc(double timestamp, double ttc, double closing_speed = 10.0)
{
  const double range = ttc * closing_speed;
  return MakeInput(timestamp, 0.0, {MakeObstacle(7, range, closing_speed)});
}

TEST(AebFsmTest, StartsInNormal)
{
  AebFsm fsm;
  EXPECT_EQ(AebState::kNormal, fsm.state());
  EXPECT_FALSE(fsm.overridden());
}

TEST(AebFsmTest, ToStringCoversEveryState)
{
  EXPECT_STREQ("NORMAL", ToString(AebState::kNormal));
  EXPECT_STREQ("FORWARD_COLLISION_WARNING", ToString(AebState::kForwardCollisionWarning));
  EXPECT_STREQ("PARTIAL_BRAKING", ToString(AebState::kPartialBraking));
  EXPECT_STREQ("FULL_EMERGENCY_BRAKE", ToString(AebState::kFullEmergencyBrake));
}

TEST(AebFsmTest, StateEnumValuesMatchThePublishedInterface)
{
  // These values are encoded into SafetyAlert.msg and the CAN status bits.
  EXPECT_EQ(0, static_cast<int>(AebState::kNormal));
  EXPECT_EQ(1, static_cast<int>(AebState::kForwardCollisionWarning));
  EXPECT_EQ(2, static_cast<int>(AebState::kPartialBraking));
  EXPECT_EQ(3, static_cast<int>(AebState::kFullEmergencyBrake));
}

TEST(AebFsmTest, StaysNormalWithNoObstacles)
{
  AebFsm fsm;
  const AebOutput output = fsm.Update(MakeInput(0.0, 25.0, {}));

  EXPECT_EQ(AebState::kNormal, output.state);
  EXPECT_DOUBLE_EQ(0.0, output.brake_pressure);
  EXPECT_DOUBLE_EQ(0.0, output.requested_deceleration);
  EXPECT_FALSE(output.state_changed);
}

TEST(AebFsmTest, EscalatesToWarningAtWarningTtc)
{
  AebFsm fsm;
  // TTC 2.0 s is inside the 2.5 s warning threshold but outside 1.6 s.
  const AebOutput output = fsm.Update(InputWithTtc(0.0, 2.0));

  EXPECT_EQ(AebState::kForwardCollisionWarning, output.state);
  EXPECT_TRUE(output.state_changed);
  EXPECT_DOUBLE_EQ(0.0, output.brake_pressure);
}

TEST(AebFsmTest, EscalatesToPartialBraking)
{
  AebFsm fsm;
  const AebOutput output = fsm.Update(InputWithTtc(0.0, 1.4));

  EXPECT_EQ(AebState::kPartialBraking, output.state);
  EXPECT_GT(output.brake_pressure, 0.0);
  EXPECT_GT(output.requested_deceleration, 0.0);
  EXPECT_LE(output.brake_pressure, fsm.config().partial_brake_pressure);
}

TEST(AebFsmTest, EscalatesToFullEmergencyBrake)
{
  AebFsm fsm;
  const AebOutput output = fsm.Update(InputWithTtc(0.0, 0.6));

  EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
  EXPECT_GT(output.requested_deceleration, 0.0);
  EXPECT_GT(output.brake_pressure, fsm.config().partial_brake_pressure);
}

TEST(AebFsmTest, EscalationSkipsIntermediateStatesInOneCycle)
{
  // A target appearing at a lethal TTC must go straight to full braking; there
  // is no time to walk the ladder.
  AebFsm fsm;
  const AebOutput output = fsm.Update(InputWithTtc(0.0, 0.4));

  EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
}

TEST(AebFsmTest, EscalatesImmediatelyWithoutWaitingForHoldTime)
{
  AebFsm fsm;
  fsm.Update(MakeInput(0.0, 25.0, {}));

  // Escalation must not be delayed even though the state just changed.
  const AebOutput output = fsm.Update(InputWithTtc(0.001, 0.5));
  EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
  EXPECT_TRUE(output.state_changed);
}

TEST(AebFsmTest, EmergencyDistanceForcesFullBraking)
{
  AebFsm fsm;
  // 3 m away is inside the 4 m emergency distance, so full braking applies
  // even though the closing speed alone would only give a warning.
  AebObstacle obstacle = MakeObstacle(3, 3.0, 1.0);
  const AebOutput output = fsm.Update(MakeInput(0.0, 0.0, {obstacle}));

  EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
}

TEST(AebFsmTest, DeEscalationIsBlockedByHoldTime)
{
  AebFsm fsm;
  fsm.Update(InputWithTtc(0.0, 0.5));
  ASSERT_EQ(AebState::kFullEmergencyBrake, fsm.state());

  // The threat is gone, but only 0.1 s has passed.
  const AebOutput output = fsm.Update(MakeInput(0.1, 25.0, {}));
  EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
  EXPECT_FALSE(output.state_changed);
}

TEST(AebFsmTest, DeEscalatesToNormalAfterHoldTime)
{
  AebFsm fsm;
  fsm.Update(InputWithTtc(0.0, 0.5));
  ASSERT_EQ(AebState::kFullEmergencyBrake, fsm.state());

  const AebOutput output = fsm.Update(MakeInput(1.0, 25.0, {}));
  EXPECT_EQ(AebState::kNormal, output.state);
  EXPECT_TRUE(output.state_changed);
  EXPECT_DOUBLE_EQ(0.0, output.brake_pressure);
}

TEST(AebFsmTest, DeEscalationRequiresTheReleaseMargin)
{
  AebFsm fsm;
  fsm.Update(InputWithTtc(0.0, 0.5));
  ASSERT_EQ(AebState::kFullEmergencyBrake, fsm.state());

  // A TTC of 0.9 s clears the 0.8 s escalation threshold but not the 1.2 s
  // release threshold, so the state must be held.
  const AebOutput held = fsm.Update(InputWithTtc(1.0, 0.9));
  EXPECT_EQ(AebState::kFullEmergencyBrake, held.state);

  // 1.5 s clears the release margin and drops to PARTIAL_BRAKING.
  const AebOutput released = fsm.Update(InputWithTtc(2.0, 1.5));
  EXPECT_EQ(AebState::kPartialBraking, released.state);
}

TEST(AebFsmTest, DoesNotChatterWhenTtcHoversOnAThreshold)
{
  AebFsm fsm;
  // Oscillate TTC around the 1.6 s partial-braking threshold.
  const double ttc_values[] = {1.55, 1.65, 1.55, 1.65, 1.55, 1.65, 1.55, 1.65};
  std::size_t changes = 0;
  AebState previous = fsm.state();

  double timestamp = 0.0;
  for (const double ttc_value : ttc_values) {
    timestamp += 0.1;
    const AebOutput output = fsm.Update(InputWithTtc(timestamp, ttc_value));
    if (output.state != previous) {
      ++changes;
      previous = output.state;
    }
  }

  // Exactly one escalation; the release margin suppresses the oscillation.
  EXPECT_EQ(1U, changes);
  EXPECT_EQ(AebState::kPartialBraking, fsm.state());
}

TEST(AebFsmTest, FullBrakingDecelerationExceedsPartial)
{
  AebConfig config;
  AebFsm partial(config);
  AebFsm full(config);

  const AebOutput partial_output = partial.Update(InputWithTtc(0.0, 1.4));
  const AebOutput full_output = full.Update(InputWithTtc(0.0, 0.6));

  EXPECT_GT(full_output.requested_deceleration, partial_output.requested_deceleration);
  EXPECT_GT(full_output.brake_pressure, partial_output.brake_pressure);
}

TEST(AebFsmTest, RequestedDecelerationNeverExceedsTheActuatorLimit)
{
  AebFsm fsm;
  // A very close, very fast target demands more than the brake system can give.
  AebObstacle obstacle = MakeObstacle(1, 6.0, 40.0);
  const AebOutput output = fsm.Update(MakeInput(0.0, 40.0, {obstacle}));

  EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
  EXPECT_LE(output.requested_deceleration, fsm.config().max_deceleration + 1e-9);
  EXPECT_LE(output.brake_pressure, 1.0);
}

TEST(AebFsmTest, RequestedDecelerationCoversTheGeometricRequirement)
{
  AebFsm fsm;
  AebObstacle obstacle = MakeObstacle(1, 12.0, 20.0);
  const AebOutput output = fsm.Update(MakeInput(0.0, 20.0, {obstacle}));

  ASSERT_EQ(AebState::kFullEmergencyBrake, output.state);
  // 20 m/s must be shed within (12 - 2) m: 400 / 20 = 20 m/s^2 requested, but
  // the actuator limit caps it at max_deceleration.
  EXPECT_NEAR(fsm.config().max_deceleration, output.requested_deceleration, 1e-9);
}

TEST(AebFsmTest, OverrideForcesTheRequestedState)
{
  AebFsm fsm;
  fsm.Override(AebState::kFullEmergencyBrake);

  const AebOutput output = fsm.Update(MakeInput(0.0, 25.0, {}));
  EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
  EXPECT_TRUE(output.overridden);
  EXPECT_GT(output.brake_pressure, 0.0);
}

TEST(AebFsmTest, OverrideSurvivesAnUnthreateningScene)
{
  AebFsm fsm;
  fsm.Override(AebState::kPartialBraking);

  double timestamp = 0.0;
  for (int step = 0; step < 10; ++step) {
    timestamp += 0.1;
    const AebOutput output = fsm.Update(MakeInput(timestamp, 25.0, {}));
    EXPECT_EQ(AebState::kPartialBraking, output.state) << "step " << step;
  }
}

TEST(AebFsmTest, ReleaseOverrideRestoresAutomaticControl)
{
  AebFsm fsm;
  fsm.Override(AebState::kFullEmergencyBrake);
  fsm.Update(MakeInput(0.0, 25.0, {}));

  fsm.ReleaseOverride();
  EXPECT_FALSE(fsm.overridden());

  const AebOutput output = fsm.Update(MakeInput(1.0, 25.0, {}));
  EXPECT_EQ(AebState::kNormal, output.state);
  EXPECT_FALSE(output.overridden);
}

TEST(AebFsmTest, OverrideToNormalCommandsNoBraking)
{
  AebFsm fsm;
  fsm.Override(AebState::kNormal);

  const AebOutput output = fsm.Update(MakeInput(0.0, 25.0, {}));
  EXPECT_EQ(AebState::kNormal, output.state);
  EXPECT_DOUBLE_EQ(0.0, output.brake_pressure);
  EXPECT_DOUBLE_EQ(0.0, output.requested_deceleration);
}

TEST(AebFsmTest, ResetClearsStateAndOverride)
{
  AebFsm fsm;
  fsm.Override(AebState::kFullEmergencyBrake);
  fsm.Update(MakeInput(0.0, 25.0, {}));
  ASSERT_EQ(AebState::kFullEmergencyBrake, fsm.state());

  fsm.Reset();
  EXPECT_EQ(AebState::kNormal, fsm.state());
  EXPECT_FALSE(fsm.overridden());

  const AebOutput output = fsm.Update(MakeInput(0.0, 25.0, {}));
  EXPECT_EQ(AebState::kNormal, output.state);
}

TEST(AebFsmTest, EscalationTtcMatchesConfiguredThresholds)
{
  const AebConfig config;
  AebFsm fsm(config);

  EXPECT_DOUBLE_EQ(config.warning_ttc,
    fsm.EscalationTtc(AebState::kForwardCollisionWarning));
  EXPECT_DOUBLE_EQ(config.partial_braking_ttc,
    fsm.EscalationTtc(AebState::kPartialBraking));
  EXPECT_DOUBLE_EQ(config.full_braking_ttc,
    fsm.EscalationTtc(AebState::kFullEmergencyBrake));
}

TEST(AebFsmTest, ReleaseTtcAddsTheConfiguredMargin)
{
  const AebConfig config;
  AebFsm fsm(config);

  EXPECT_DOUBLE_EQ(config.full_braking_ttc + config.release_ttc_margin,
    fsm.ReleaseTtc(AebState::kFullEmergencyBrake));
  EXPECT_GT(fsm.ReleaseTtc(AebState::kFullEmergencyBrake),
    fsm.EscalationTtc(AebState::kFullEmergencyBrake));
}

TEST(AebFsmTest, ReportsTheSelectedTargetOnTheOutput)
{
  AebFsm fsm;
  const AebOutput output = fsm.Update(InputWithTtc(0.0, 1.4));
  EXPECT_EQ(7U, output.target_track_id);
  EXPECT_GT(output.distance, 0.0);
  EXPECT_GT(output.closing_speed, 0.0);
}

TEST(AebFsmTest, FullEscalationLadderIsReachedAsTtcShrinks)
{
  AebConfig config;
  config.min_state_duration = 0.0;
  AebFsm fsm(config);

  const AebOutput warning = fsm.Update(InputWithTtc(0.0, 2.0));
  EXPECT_EQ(AebState::kForwardCollisionWarning, warning.state);

  const AebOutput partial = fsm.Update(InputWithTtc(0.1, 1.4));
  EXPECT_EQ(AebState::kPartialBraking, partial.state);

  const AebOutput full = fsm.Update(InputWithTtc(0.2, 0.5));
  EXPECT_EQ(AebState::kFullEmergencyBrake, full.state);
}

TEST(AebFsmTest, IgnoredWhenNoObstacleIsInLane)
{
  AebFsm fsm;
  AebObstacle lateral = MakeObstacle(1, 30.0, 10.0);
  lateral.y = 8.0;
  lateral.range = std::hypot(lateral.x, lateral.y);

  const AebOutput output = fsm.Update(MakeInput(0.0, 25.0, {lateral}));
  EXPECT_EQ(AebState::kNormal, output.state);
}

}  // namespace
}  // namespace sil_aeb_controller
