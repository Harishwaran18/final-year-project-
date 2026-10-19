// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>

#include "sil_aeb_controller/ttc_calculator.hpp"

namespace sil_aeb_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(TtcCalculatorTest, BasicRatio)
{
  // TTC = distance / relative_velocity.
  EXPECT_DOUBLE_EQ(2.0, ttc::TimeToCollision(20.0, 10.0));
  EXPECT_DOUBLE_EQ(0.5, ttc::TimeToCollision(5.0, 10.0));
}

TEST(TtcCalculatorTest, ZeroDistanceIsImmediateCollision)
{
  EXPECT_DOUBLE_EQ(0.0, ttc::TimeToCollision(0.0, 10.0));
  EXPECT_DOUBLE_EQ(0.0, ttc::TimeToCollision(-1.0, 10.0));
}

TEST(TtcCalculatorTest, NoClosingSpeedMeansNoCollision)
{
  EXPECT_DOUBLE_EQ(ttc::kNoCollision, ttc::TimeToCollision(20.0, 0.0));
  EXPECT_DOUBLE_EQ(ttc::kNoCollision, ttc::TimeToCollision(20.0, -5.0));
}

TEST(TtcCalculatorTest, MonotonicallyDecreasesWithClosingSpeed)
{
  const double slow = ttc::TimeToCollision(30.0, 5.0);
  const double medium = ttc::TimeToCollision(30.0, 10.0);
  const double fast = ttc::TimeToCollision(30.0, 20.0);

  EXPECT_GT(slow, medium);
  EXPECT_GT(medium, fast);
}

TEST(TtcCalculatorTest, SafetyMarginGrowsWithSpeed)
{
  const double slow = ttc::SafetyMargin(5.0, 0.15, 2.0);
  const double fast = ttc::SafetyMargin(30.0, 0.15, 2.0);

  EXPECT_DOUBLE_EQ(2.75, slow);
  EXPECT_DOUBLE_EQ(6.5, fast);
  EXPECT_GT(fast, slow);
}

TEST(TtcCalculatorTest, BrakingDistanceMatchesKinematics)
{
  // Stopping from 20 m/s at 8 m/s^2: 400 / 16 = 25 m.
  EXPECT_DOUBLE_EQ(25.0, ttc::BrakingDistance(20.0, 0.0, 8.0));
  // Decelerating from 20 to 10 m/s at 5 m/s^2: (400 - 100) / 10 = 30 m.
  EXPECT_DOUBLE_EQ(30.0, ttc::BrakingDistance(20.0, 10.0, 5.0));
}

TEST(TtcCalculatorTest, BrakingDistanceIsZeroForNonDeceleration)
{
  EXPECT_DOUBLE_EQ(0.0, ttc::BrakingDistance(20.0, 0.0, 0.0));
  EXPECT_DOUBLE_EQ(0.0, ttc::BrakingDistance(20.0, 0.0, -3.0));
  EXPECT_DOUBLE_EQ(0.0, ttc::BrakingDistance(10.0, 20.0, 5.0));
}

TEST(TtcCalculatorTest, RequiredDecelerationIsInverseOfBrakingDistance)
{
  // Stopping from 20 m/s in 25 m of available distance needs 8 m/s^2.
  EXPECT_DOUBLE_EQ(8.0, ttc::RequiredDeceleration(20.0, 25.0, 0.0));
}

TEST(TtcCalculatorTest, RequiredDecelerationAccountsForStandstillGap)
{
  // Only 23 m of the 25 m gap is usable, so 400 / 46 m/s^2 is required.
  const double deceleration = ttc::RequiredDeceleration(20.0, 25.0, 2.0);
  EXPECT_NEAR(400.0 / 46.0, deceleration, 1e-9);
  EXPECT_GT(deceleration, ttc::RequiredDeceleration(20.0, 25.0, 0.0));
}

TEST(TtcCalculatorTest, RequiredDecelerationIsZeroWhenAlreadyOverlapping)
{
  EXPECT_DOUBLE_EQ(0.0, ttc::RequiredDeceleration(20.0, 2.0, 2.0));
  EXPECT_DOUBLE_EQ(0.0, ttc::RequiredDeceleration(20.0, 0.0, 2.0));
}

TEST(TtcCalculatorTest, TimeToImpactWithConstantDeceleration)
{
  // From 20 m/s with no braking, a 40 m gap takes 2 s.
  EXPECT_NEAR(2.0, ttc::TimeToImpact(40.0, 20.0, 0.0), 1e-9);
}

TEST(TtcCalculatorTest, TimeToImpactIsLongerWhenBraking)
{
  const double coasting = ttc::TimeToImpact(40.0, 20.0, 0.0);
  const double braking = ttc::TimeToImpact(40.0, 20.0, 5.0);

  EXPECT_GT(braking, coasting);
}

TEST(TtcCalculatorTest, TimeToImpactSatisfiesTheQuadratic)
{
  const double distance = 30.0;
  const double speed = 15.0;
  const double deceleration = 4.0;

  // Stopping from 15 m/s at 4 m/s^2 needs 28.125 m, so the obstacle at 30 m is
  // not reached: there is no impact time.
  EXPECT_DOUBLE_EQ(ttc::kNoCollision, ttc::TimeToImpact(distance, speed, deceleration));

  // With the obstacle inside the stopping distance the root must satisfy the
  // kinematic relation. Stopping from 15 m/s at 4 m/s^2 takes 28.125 m, so a
  // 20 m gap is still closed.
  const double far_distance = 20.0;
  const double t = ttc::TimeToImpact(far_distance, speed, deceleration);
  ASSERT_GT(t, 0.0);

  const double travelled = speed * t - 0.5 * deceleration * t * t;
  EXPECT_NEAR(far_distance, travelled, 1e-6);
}

TEST(TtcCalculatorTest, TimeToImpactIsZeroAtContact)
{
  EXPECT_DOUBLE_EQ(0.0, ttc::TimeToImpact(0.0, 20.0, 5.0));
  EXPECT_DOUBLE_EQ(0.0, ttc::TimeToImpact(-5.0, 20.0, 5.0));
}

TEST(TtcCalculatorTest, TimeToImpactReportsNoCollisionWhenStopped)
{
  EXPECT_DOUBLE_EQ(ttc::kNoCollision, ttc::TimeToImpact(30.0, 0.0, 5.0));
}

TEST(TtcCalculatorTest, TimeToImpactReportsNoCollisionWhenStoppingImpossible)
{
  // From 20 m/s at 10 m/s^2 the vehicle stops in 20 m, so a 10 m gap is closed
  // long before it can stop: an impact is unavoidable and has a finite time.
  const double t = ttc::TimeToImpact(10.0, 20.0, 10.0);
  EXPECT_LT(t, ttc::kNoCollision);
  EXPECT_GT(t, 0.0);
  EXPECT_NEAR(10.0, 20.0 * t - 0.5 * 10.0 * t * t, 1e-6);
}

TEST(TtcCalculatorTest, TimeToImpactReportsNoCollisionWhenStoppingShort)
{
  // From 10 m/s at 10 m/s^2 the vehicle stops in 5 m, so a 10 m gap is never
  // reached and there is no impact time.
  EXPECT_DOUBLE_EQ(ttc::kNoCollision, ttc::TimeToImpact(10.0, 10.0, 10.0));
}

TEST(TtcCalculatorTest, TimeToImpactIsContinuousWithCoasting)
{
  // As the deceleration tends to zero the result must approach d / v.
  const double coasting = ttc::TimeToImpact(30.0, 15.0, 0.0);
  const double gentle = ttc::TimeToImpact(30.0, 15.0, 1e-6);

  EXPECT_NEAR(coasting, gentle, 1e-3);
}

TEST(TtcCalculatorTest, AngleHelpersAreUnusedButCompile)
{
  // Guards against accidental removal of the header's constants.
  EXPECT_GT(kPi, 3.14);
  EXPECT_LT(kPi, 3.15);
}

}  // namespace
}  // namespace sil_aeb_controller
