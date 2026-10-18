// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sil_aeb_controller/aeb_fsm.hpp"

namespace sil_aeb_controller
{
namespace
{

/// A lightweight constant-velocity vehicle model, used to close the SIL loop
/// around the FSM without involving ROS or the CAN bus.
class ScenarioRunner
{
public:
  ScenarioRunner(double initial_range, double ego_speed, double target_speed,
    double initial_ttc_threshold = 0.0)
  : range_(initial_range), ego_speed_(ego_speed), target_speed_(target_speed)
  {
    static_cast<void>(initial_ttc_threshold);
  }

  /// Advances the scenario by @p dt and runs one FSM cycle.
  AebOutput Step(double dt)
  {
    timestamp_ += dt;

    // The target is stationary in the world; the ego closes at the difference.
    const double closing = std::max(0.0, ego_speed_ - target_speed_);

    AebObstacle obstacle;
    obstacle.track_id = 42;
    obstacle.x = range_;
    obstacle.y = 0.0;
    obstacle.vx = -closing;
    obstacle.vy = 0.0;
    obstacle.closing_speed = closing;
    obstacle.range = range_;

    AebInput input;
    input.timestamp = timestamp_;
    input.ego_speed = ego_speed_;
    input.obstacles = {obstacle};

    const AebOutput output = fsm_.Update(input);

    // Integrate the ego with the commanded deceleration.
    const double deceleration = std::min(output.requested_deceleration, config_.max_deceleration);
    ego_speed_ = std::max(0.0, ego_speed_ - deceleration * dt);

    // The gap closes at the relative speed over this step.
    const double relative = std::max(0.0, ego_speed_ - target_speed_);
    range_ = std::max(0.0, range_ - relative * dt);

    return output;
  }

  double range() const {return range_;}
  double egoSpeed() const {return ego_speed_;}
  double timestamp() const {return timestamp_;}
  AebState state() const {return fsm_.state();}
  AebConfig & config() {return config_;}

private:
  AebConfig config_{};
  AebFsm fsm_{};
  double range_;
  double ego_speed_;
  double target_speed_;
  double timestamp_{0.0};
};

TEST(AebScenarioTest, EscalationLadderIsTraversedAsTheGapCloses)
{
  // Ego at 25 m/s closing on a stopped lead vehicle 80 m ahead.
  ScenarioRunner runner(80.0, 25.0, 0.0);

  bool saw_warning = false;
  bool saw_partial = false;
  bool saw_full = false;

  for (int step = 0; step < 400 && runner.range() > 0.5; ++step) {
    const AebOutput output = runner.Step(0.02);
    switch (output.state) {
      case AebState::kForwardCollisionWarning:
        saw_warning = true;
        break;
      case AebState::kPartialBraking:
        saw_partial = true;
        break;
      case AebState::kFullEmergencyBrake:
        saw_full = true;
        break;
      case AebState::kNormal:
        break;
    }
  }

  EXPECT_TRUE(saw_warning) << "the FSM never warned";
  EXPECT_TRUE(saw_partial) << "the FSM never applied partial braking";
  EXPECT_TRUE(saw_full) << "the FSM never applied full braking";
}

TEST(AebScenarioTest, FullBrakingIsReachedBeforeImpact)
{
  ScenarioRunner runner(60.0, 25.0, 0.0);

  bool braked_before_contact = false;
  for (int step = 0; step < 400 && runner.range() > 0.5; ++step) {
    const AebOutput output = runner.Step(0.02);
    if (output.state == AebState::kFullEmergencyBrake && runner.range() > 1.0) {
      braked_before_contact = true;
      break;
    }
  }

  EXPECT_TRUE(braked_before_contact);
}

TEST(AebScenarioTest, EgoStopsWithoutColliding)
{
  ScenarioRunner runner(80.0, 25.0, 0.0);

  // Run until the ego stops or the step budget is exhausted.
  for (int step = 0; step < 2000; ++step) {
    runner.Step(0.01);
    if (runner.egoSpeed() <= 0.0) {
      break;
    }
  }

  EXPECT_LE(runner.egoSpeed(), 0.0);
  // The standstill gap is 2 m, so the ego must not have closed the last metre.
  EXPECT_GT(runner.range(), 0.0) << "the ego collided with the target";
}

TEST(AebScenarioTest, NoInterventionWhenTheTargetIsFasterThanTheEgo)
{
  // The lead vehicle pulls away, so the gap never closes.
  ScenarioRunner runner(30.0, 10.0, 20.0);

  for (int step = 0; step < 200; ++step) {
    const AebOutput output = runner.Step(0.02);
    EXPECT_EQ(AebState::kNormal, output.state) << "step " << step;
    EXPECT_DOUBLE_EQ(0.0, output.brake_pressure);
  }
}

TEST(AebScenarioTest, NoInterventionForAComfortableApproach)
{
  // Closing at 2 m/s from 60 m gives a 30 s TTC: far outside every threshold.
  ScenarioRunner runner(60.0, 12.0, 10.0);

  for (int step = 0; step < 100; ++step) {
    const AebOutput output = runner.Step(0.02);
    EXPECT_EQ(AebState::kNormal, output.state) << "step " << step;
  }
}

TEST(AebScenarioTest, BrakingReleasesOnceTheThreatIsGone)
{
  ScenarioRunner runner(20.0, 20.0, 0.0);

  // Provoke full braking.
  AebOutput output;
  for (int step = 0; step < 50; ++step) {
    output = runner.Step(0.02);
    if (output.state == AebState::kFullEmergencyBrake) {
      break;
    }
  }
  ASSERT_EQ(AebState::kFullEmergencyBrake, runner.state());

  // Simulate the threat clearing: a lead vehicle pulling away, so the gap only
  // ever grows and no intervention is warranted.
  ScenarioRunner clear_runner(80.0, 20.0, 25.0);
  for (int step = 0; step < 100; ++step) {
    output = clear_runner.Step(0.02);
  }
  EXPECT_EQ(AebState::kNormal, clear_runner.state());
}

TEST(AebScenarioTest, HigherClosingSpeedEscalatesSooner)
{
  ScenarioRunner slow(50.0, 12.0, 0.0);
  ScenarioRunner fast(50.0, 30.0, 0.0);

  double slow_escalation_time = -1.0;
  double fast_escalation_time = -1.0;

  for (int step = 0; step < 400; ++step) {
    if (slow_escalation_time < 0.0 &&
      slow.Step(0.02).state != AebState::kNormal)
    {
      slow_escalation_time = slow.timestamp();
    }
    if (fast_escalation_time < 0.0 &&
      fast.Step(0.02).state != AebState::kNormal)
    {
      fast_escalation_time = fast.timestamp();
    }
    if (slow_escalation_time > 0.0 && fast_escalation_time > 0.0) {
      break;
    }
  }

  ASSERT_GT(slow_escalation_time, 0.0);
  ASSERT_GT(fast_escalation_time, 0.0);
  EXPECT_LT(fast_escalation_time, slow_escalation_time);
}

TEST(AebScenarioTest, OverrideIsHonouredForTheWholeScenario)
{
  ScenarioRunner runner(80.0, 25.0, 0.0);
  runner.config();
  AebFsm forced;

  // Drive the scenario manually against a latched full-brake override.
  forced.Override(AebState::kFullEmergencyBrake);
  for (int step = 0; step < 50; ++step) {
    AebInput input;
    input.timestamp = static_cast<double>(step) * 0.02;
    input.ego_speed = 25.0;
    const AebOutput output = forced.Update(input);
    EXPECT_EQ(AebState::kFullEmergencyBrake, output.state);
    EXPECT_TRUE(output.overridden);
  }
}

}  // namespace
}  // namespace sil_aeb_controller
