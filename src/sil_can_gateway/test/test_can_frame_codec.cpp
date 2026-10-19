// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>

#include "sil_can_gateway/can_frame_codec.hpp"

namespace sil_can_gateway
{
namespace
{

BrakeCommand MakeCommand(double deceleration, double pressure, std::uint8_t state,
  std::uint16_t track_id, bool changed = false, bool overridden = false,
  bool valid = true)
{
  BrakeCommand command;
  command.requested_deceleration = deceleration;
  command.brake_pressure = pressure;
  command.state = state;
  command.target_track_id = track_id;
  command.state_changed = changed;
  command.overridden = overridden;
  command.valid = valid;
  return command;
}

TEST(CanFrameCodecTest, BrakeFrameHasTheExpectedIdentity)
{
  const CanFrame frame = EncodeBrakeCommand(MakeCommand(5.0, 0.5, 2, 17), 0);

  EXPECT_EQ(protocol::kBrakeCommandId, frame.can_id);
  EXPECT_EQ(protocol::kDlc, frame.dlc);
  EXPECT_EQ(8U, frame.data.size());
}

TEST(CanFrameCodecTest, DecelerationIsLittleEndianAtBytesZeroAndOne)
{
  // 5.0 m/s^2 at 0.001 per count is 5000 counts == 0x1388.
  const CanFrame frame = EncodeBrakeCommand(MakeCommand(5.0, 0.0, 0, 0), 0);

  EXPECT_EQ(0x88U, frame.data[0]) << "low byte must come first";
  EXPECT_EQ(0x13U, frame.data[1]);
}

TEST(CanFrameCodecTest, TargetTrackIdIsLittleEndianAtBytesFourAndFive)
{
  // 0x1234 -> low byte 0x34 at index 4, high byte 0x12 at index 5.
  const CanFrame frame = EncodeBrakeCommand(MakeCommand(0.0, 0.0, 0, 0x1234), 0);

  EXPECT_EQ(0x34U, frame.data[4]);
  EXPECT_EQ(0x12U, frame.data[5]);
}

TEST(CanFrameCodecTest, PressureIsAtByteTwo)
{
  const CanFrame frame = EncodeBrakeCommand(MakeCommand(0.0, 1.0, 3, 0), 0);
  EXPECT_EQ(250U, frame.data[2]);
}

TEST(CanFrameCodecTest, StatusBitsArePackedCorrectly)
{
  // State 3 occupies bits [1:0]; changed is bit 2; override is bit 3;
  // valid is bit 4. 3 | 0x04 | 0x08 | 0x10 == 0x1F.
  const CanFrame frame = EncodeBrakeCommand(MakeCommand(1.0, 0.0, 3, 0, true, true, true), 0);

  EXPECT_EQ(0x1FU, frame.data[3]);
  EXPECT_EQ(0x03U, frame.data[3] & protocol::kStatusStateMask);
  EXPECT_NE(0U, frame.data[3] & protocol::kStatusChangedBit);
  EXPECT_NE(0U, frame.data[3] & protocol::kStatusOverrideBit);
  EXPECT_NE(0U, frame.data[3] & protocol::kStatusValidBit);
}

TEST(CanFrameCodecTest, StatusReservedBitsStayClear)
{
  // Only four bits are defined, so the top three must never be set by the
  // encoder regardless of the state value.
  for (std::uint8_t state = 0; state <= 3; ++state) {
    const CanFrame frame = EncodeBrakeCommand(
      MakeCommand(1.0, 0.5, state, 0, true, true, true), 0);
    EXPECT_EQ(0U, frame.data[3] & protocol::kStatusReservedMask)
      << "state " << static_cast<int>(state);
  }
}

TEST(CanFrameCodecTest, RollingCounterOccupiesTheHighNibble)
{
  for (std::uint8_t counter = 0; counter < 16; ++counter) {
    const CanFrame frame = EncodeBrakeCommand(MakeCommand(1.0, 0.0, 0, 0), counter);
    EXPECT_EQ(counter, FrameCounter(frame)) << "counter " << static_cast<int>(counter);
    EXPECT_EQ(static_cast<std::uint8_t>(counter << protocol::kCounterShift),
      static_cast<std::uint8_t>(frame.data[6] & 0xF0U));
  }
}

TEST(CanFrameCodecTest, CounterIsMaskedToFourBits)
{
  // 0x1F has bit 4 set, which must be discarded rather than bleeding into the
  // reserved nibble.
  const CanFrame frame = EncodeBrakeCommand(MakeCommand(1.0, 0.0, 0, 0), 0x1F);
  EXPECT_EQ(0x0FU, FrameCounter(frame));
}

TEST(CanFrameCodecTest, CrcIsTheLastByte)
{
  const CanFrame frame = EncodeBrakeCommand(MakeCommand(3.5, 0.35, 2, 9), 4);
  EXPECT_EQ(FrameChecksum(frame.data), frame.data[protocol::kCrcByteIndex]);
}

TEST(CanFrameCodecTest, BrakeCommandRoundTrips)
{
  const BrakeCommand original = MakeCommand(6.5, 0.8, 3, 4242, true, false, true);

  const CanFrame frame = EncodeBrakeCommand(original, 7);
  BrakeCommand decoded;
  ASSERT_TRUE(DecodeBrakeCommand(frame, decoded));

  EXPECT_NEAR(original.requested_deceleration, decoded.requested_deceleration,
    protocol::kDecelerationScale);
  EXPECT_NEAR(original.brake_pressure, decoded.brake_pressure,
    protocol::kPressureScale);
  EXPECT_EQ(original.state, decoded.state);
  EXPECT_EQ(original.target_track_id, decoded.target_track_id);
  EXPECT_EQ(original.state_changed, decoded.state_changed);
  EXPECT_EQ(original.overridden, decoded.overridden);
  EXPECT_TRUE(decoded.valid);
}

TEST(CanFrameCodecTest, BrakeCommandRoundTripsAcrossTheWholeStateRange)
{
  for (std::uint8_t state = 0; state <= 3; ++state) {
    const CanFrame frame = EncodeBrakeCommand(MakeCommand(2.0, 0.25, state, state), 0);
    BrakeCommand decoded;
    ASSERT_TRUE(DecodeBrakeCommand(frame, decoded));
    EXPECT_EQ(state, decoded.state);
    EXPECT_EQ(state, decoded.target_track_id);
  }
}

TEST(CanFrameCodecTest, DecodeRejectsAWrongIdentifier)
{
  CanFrame frame = EncodeBrakeCommand(MakeCommand(5.0, 0.5, 2, 1), 0);
  frame.can_id = protocol::kTargetSpeedId;

  BrakeCommand decoded;
  EXPECT_FALSE(DecodeBrakeCommand(frame, decoded));
}

TEST(CanFrameCodecTest, DecodeRejectsAWrongDlc)
{
  CanFrame frame = EncodeBrakeCommand(MakeCommand(5.0, 0.5, 2, 1), 0);
  frame.dlc = 6;

  BrakeCommand decoded;
  EXPECT_FALSE(DecodeBrakeCommand(frame, decoded));
}

TEST(CanFrameCodecTest, DecodeRejectsACorruptedPayload)
{
  CanFrame frame = EncodeBrakeCommand(MakeCommand(5.0, 0.5, 2, 1), 0);
  frame.data[0] = static_cast<std::uint8_t>(frame.data[0] ^ 0x01);

  BrakeCommand decoded;
  EXPECT_FALSE(DecodeBrakeCommand(frame, decoded)) << "the CRC must catch this";
}

TEST(CanFrameCodecTest, DecodeRejectsACorruptedCrc)
{
  CanFrame frame = EncodeBrakeCommand(MakeCommand(5.0, 0.5, 2, 1), 0);
  frame.data[protocol::kCrcByteIndex] =
    static_cast<std::uint8_t>(frame.data[protocol::kCrcByteIndex] ^ 0xFF);

  BrakeCommand decoded;
  EXPECT_FALSE(DecodeBrakeCommand(frame, decoded));
}

TEST(CanFrameCodecTest, DecodeDoesNotMutateTheOutputOnFailure)
{
  CanFrame frame = EncodeBrakeCommand(MakeCommand(5.0, 0.5, 2, 1), 0);
  frame.data[3] = static_cast<std::uint8_t>(frame.data[3] ^ 0x10);

  BrakeCommand decoded;
  decoded.state = 99;
  EXPECT_FALSE(DecodeBrakeCommand(frame, decoded));
  EXPECT_EQ(99, decoded.state) << "a failed decode must leave the output untouched";
}

TEST(CanFrameCodecTest, TargetSpeedRoundTrips)
{
  TargetSpeedCommand original;
  original.target_speed = 12.5;
  original.ego_speed = -3.25;
  // The deceleration field is coarse (0.01 m/s^2 per count), so use a value on
  // the grid to keep the round trip exact.
  original.deceleration = 2.55;
  original.brake_pressure = 0.6;

  const CanFrame frame = EncodeTargetSpeed(original, 3);
  EXPECT_EQ(protocol::kTargetSpeedId, frame.can_id);
  EXPECT_EQ(protocol::kDlc, frame.dlc);

  TargetSpeedCommand decoded;
  ASSERT_TRUE(DecodeTargetSpeed(frame, decoded));
  EXPECT_NEAR(original.target_speed, decoded.target_speed, protocol::kSpeedScale);
  EXPECT_NEAR(original.ego_speed, decoded.ego_speed, protocol::kSpeedScale);
  EXPECT_NEAR(original.deceleration, decoded.deceleration,
    protocol::kCoarseDecelerationScale);
  EXPECT_NEAR(original.brake_pressure, decoded.brake_pressure,
    protocol::kPressureScale);
}

TEST(CanFrameCodecTest, CoarseDecelerationFieldSaturatesAtItsMaximum)
{
  // 4.5 m/s^2 would need 450 counts, far beyond the 8-bit field, so it clamps
  // to 255 counts == 2.55 m/s^2 rather than wrapping to 194 counts.
  TargetSpeedCommand command;
  command.deceleration = 4.5;

  const CanFrame frame = EncodeTargetSpeed(command, 0);
  EXPECT_EQ(255U, frame.data[4]);

  TargetSpeedCommand decoded;
  ASSERT_TRUE(DecodeTargetSpeed(frame, decoded));
  EXPECT_NEAR(2.55, decoded.deceleration, 1e-9);
}

TEST(CanFrameCodecTest, NegativeSpeedsSurviveSignExtension)
{
  TargetSpeedCommand original;
  original.target_speed = -8.0;
  original.ego_speed = -1.0;

  TargetSpeedCommand decoded;
  ASSERT_TRUE(DecodeTargetSpeed(EncodeTargetSpeed(original, 0), decoded));
  EXPECT_LT(decoded.target_speed, 0.0);
  EXPECT_LT(decoded.ego_speed, 0.0);
  EXPECT_NEAR(-8.0, decoded.target_speed, protocol::kSpeedScale);
  EXPECT_NEAR(-1.0, decoded.ego_speed, protocol::kSpeedScale);
}

TEST(CanFrameCodecTest, TargetSpeedIsLittleEndian)
{
  TargetSpeedCommand command;
  command.target_speed = 25.0;  // 25000 counts == 0x61A8
  const CanFrame frame = EncodeTargetSpeed(command, 0);

  EXPECT_EQ(0xA8U, frame.data[0]);
  EXPECT_EQ(0x61U, frame.data[1]);
}

TEST(CanFrameCodecTest, TargetSpeedRejectsTheWrongIdentifier)
{
  CanFrame frame = EncodeTargetSpeed(TargetSpeedCommand{}, 0);
  frame.can_id = protocol::kBrakeCommandId;

  TargetSpeedCommand decoded;
  EXPECT_FALSE(DecodeTargetSpeed(frame, decoded));
}

TEST(CanFrameCodecTest, TargetSpeedRejectsACorruptedPayload)
{
  CanFrame frame = EncodeTargetSpeed(TargetSpeedCommand{}, 0);
  frame.data[4] = static_cast<std::uint8_t>(frame.data[4] ^ 0x40);

  TargetSpeedCommand decoded;
  EXPECT_FALSE(DecodeTargetSpeed(frame, decoded));
}

TEST(CanFrameCodecTest, ZeroCommandEncodesToZerosExceptTheCrc)
{
  const CanFrame frame = EncodeBrakeCommand(BrakeCommand{}, 0);

  for (std::size_t index = 0; index < protocol::kCrcByteIndex; ++index) {
    EXPECT_EQ(0U, frame.data[index]) << "byte " << index;
  }
  EXPECT_EQ(FrameChecksum(frame.data), frame.data[protocol::kCrcByteIndex]);
}

TEST(CanFrameCodecTest, FullBrakeCommandProducesTheDocumentedByteLayout)
{
  // Pin the exact wire format so a future refactor cannot silently change the
  // protocol that the HIL bench depends on.
  BrakeCommand command;
  command.requested_deceleration = 9.0;  // 9000 counts == 0x2328
  command.brake_pressure = 1.0;          // 250 counts == 0xFA
  command.state = 3;
  command.target_track_id = 0x0102;      // low 0x02, high 0x01
  command.state_changed = true;
  command.overridden = false;
  command.valid = true;

  const CanFrame frame = EncodeBrakeCommand(command, 5);

  EXPECT_EQ(protocol::kBrakeCommandId, frame.can_id);
  EXPECT_EQ(0x28U, frame.data[0]);
  EXPECT_EQ(0x23U, frame.data[1]);
  EXPECT_EQ(0xFAU, frame.data[2]);
  // state 3 | changed 0x04 | valid 0x10 == 0x17
  EXPECT_EQ(0x17U, frame.data[3]);
  EXPECT_EQ(0x02U, frame.data[4]);
  EXPECT_EQ(0x01U, frame.data[5]);
  EXPECT_EQ(0x50U, frame.data[6]);  // counter 5 in the high nibble
  EXPECT_EQ(FrameChecksum(frame.data), frame.data[7]);
}

TEST(CanFrameCodecTest, DecodedFrameEqualsOriginalForTheDocumentedLayout)
{
  BrakeCommand command;
  command.requested_deceleration = 9.0;
  command.brake_pressure = 1.0;
  command.state = 3;
  command.target_track_id = 0x0102;
  command.valid = true;

  BrakeCommand decoded;
  ASSERT_TRUE(DecodeBrakeCommand(EncodeBrakeCommand(command, 5), decoded));
  EXPECT_NEAR(9.0, decoded.requested_deceleration, 1e-9);
  EXPECT_NEAR(1.0, decoded.brake_pressure, 1e-9);
  EXPECT_EQ(3, decoded.state);
  EXPECT_EQ(0x0102, decoded.target_track_id);
}

TEST(CanFrameCodecTest, FrameEqualityComparesEveryField)
{
  const CanFrame a = EncodeBrakeCommand(MakeCommand(1.0, 0.1, 1, 1), 1);
  const CanFrame b = EncodeBrakeCommand(MakeCommand(1.0, 0.1, 1, 1), 1);
  const CanFrame c = EncodeBrakeCommand(MakeCommand(1.0, 0.1, 1, 1), 2);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
}

}  // namespace
}  // namespace sil_can_gateway
