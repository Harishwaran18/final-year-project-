// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "sil_can_gateway/can_frame_codec.hpp"

namespace sil_can_gateway
{
namespace
{

TEST(ScalingTest, UnsignedRoundsToNearestCount)
{
  // 0.0123 m/s^2 at 0.001 m/s^2 per count is 12.3 counts, which rounds to 12.
  EXPECT_EQ(12U, ScaleToUnsigned(0.0123, 0.001, 65535U));
  // 12.6 counts rounds to 13.
  EXPECT_EQ(13U, ScaleToUnsigned(0.0126, 0.001, 65535U));
  // 12.5 counts rounds away from zero to 13.
  EXPECT_EQ(13U, ScaleToUnsigned(0.0125, 0.001, 65535U));
}

TEST(ScalingTest, UnsignedSaturatesAtTheMaximum)
{
  EXPECT_EQ(250U, ScaleToUnsigned(5.0, 0.004, 250U));
  EXPECT_EQ(250U, ScaleToUnsigned(1.0, 0.004, 250U));
}

TEST(ScalingTest, UnsignedClampsNonPositiveAndNonFiniteInput)
{
  EXPECT_EQ(0U, ScaleToUnsigned(0.0, 0.001, 65535U));
  EXPECT_EQ(0U, ScaleToUnsigned(-5.0, 0.001, 65535U));
  EXPECT_EQ(0U, ScaleToUnsigned(
    std::numeric_limits<double>::quiet_NaN(), 0.001, 65535U));
  EXPECT_EQ(0U, ScaleToUnsigned(
    std::numeric_limits<double>::infinity(), 0.001, 65535U));
}

TEST(ScalingTest, UnsignedRejectsADegenerateScale)
{
  EXPECT_EQ(0U, ScaleToUnsigned(1.0, 0.0, 65535U));
  EXPECT_EQ(0U, ScaleToUnsigned(1.0, -1.0, 65535U));
}

TEST(ScalingTest, SignedHandlesBothDirections)
{
  EXPECT_EQ(1000, ScaleToSigned(1.0, 0.001));
  EXPECT_EQ(-1000, ScaleToSigned(-1.0, 0.001));
  EXPECT_EQ(0, ScaleToSigned(0.0, 0.001));
}

TEST(ScalingTest, SignedSaturatesAtTheInt16Limits)
{
  EXPECT_EQ(32767, ScaleToSigned(1000.0, 0.001));
  EXPECT_EQ(-32768, ScaleToSigned(-1000.0, 0.001));
}

TEST(ScalingTest, SignedClampsNonFiniteInput)
{
  EXPECT_EQ(0, ScaleToSigned(std::numeric_limits<double>::quiet_NaN(), 0.001));
  EXPECT_EQ(0, ScaleToSigned(std::numeric_limits<double>::infinity(), 0.001));
}

TEST(ScalingTest, SignedRoundTripsThroughTheDecodePath)
{
  const double speeds[] = {0.0, 1.5, -1.5, 12.345, -12.345, 30.0, -30.0};
  for (const double speed : speeds) {
    const std::int32_t counts = ScaleToSigned(speed, protocol::kSpeedScale);
    const double decoded = static_cast<double>(static_cast<std::int16_t>(counts & 0xFFFF)) *
      protocol::kSpeedScale;
    EXPECT_NEAR(speed, decoded, protocol::kSpeedScale) << "speed " << speed;
  }
}

TEST(ScalingTest, UnsignedRoundTripsThroughTheDecodePath)
{
  const double decelerations[] = {0.0, 0.5, 3.5, 9.0, 9.5, 65.0};
  for (const double deceleration : decelerations) {
    const std::uint32_t counts = ScaleToUnsigned(
      deceleration, protocol::kDecelerationScale,
      static_cast<std::uint32_t>(protocol::kDecelerationMax / protocol::kDecelerationScale));
    const double decoded = static_cast<double>(counts) * protocol::kDecelerationScale;
    EXPECT_NEAR(deceleration, decoded, protocol::kDecelerationScale)
      << "deceleration " << deceleration;
  }
}

TEST(ScalingTest, PressureSaturatesAtOneHundredPercent)
{
  // 1.0 normalised pressure is 250 counts at 0.004 per count.
  EXPECT_EQ(250U, ScaleToUnsigned(1.0, protocol::kPressureScale, protocol::kPressureMaxCount));
  // Anything above 1.0 clamps, so it cannot wrap into a lower value.
  EXPECT_EQ(250U, ScaleToUnsigned(1.5, protocol::kPressureScale, protocol::kPressureMaxCount));
  EXPECT_EQ(125U, ScaleToUnsigned(0.5, protocol::kPressureScale, protocol::kPressureMaxCount));
}

TEST(ScalingTest, DecelerationSaturationAvoidsThe16BitWrap)
{
  // 70 m/s^2 would be 70000 counts and wrap a uint16; it must saturate instead.
  const std::uint32_t counts = ScaleToUnsigned(
    70.0, protocol::kDecelerationScale, protocol::kDecelerationMaxCount);
  EXPECT_EQ(protocol::kDecelerationMaxCount, counts);
  EXPECT_LE(counts, 65535U);
}

}  // namespace
}  // namespace sil_can_gateway
