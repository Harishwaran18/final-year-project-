// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_can_gateway/can_frame_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace sil_can_gateway
{
namespace
{
/// Normalised pressure below which the count is treated as zero.
constexpr double kMinPressure = 0.0;
constexpr double kMaxPressure = 1.0;

/// Little-endian 16-bit store.
void StoreUint16(std::array<std::uint8_t, 8> & data, std::size_t offset,
  std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  data[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

/// Little-endian 16-bit load.
std::uint16_t LoadUint16(const std::array<std::uint8_t, 8> & data, std::size_t offset)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[offset]) |
    static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[offset + 1]) << 8U));
}

/// Writes the rolling counter and CRC into bytes 6 and 7.
void FinaliseFrame(std::array<std::uint8_t, 8> & data, std::uint8_t counter)
{
  const std::uint8_t high_nibble =
    static_cast<std::uint8_t>((counter & protocol::kCounterMask) << protocol::kCounterShift);
  // Preserve the reserved low nibble rather than clearing it, so a future
  // signal can occupy it without a protocol break.
  data[protocol::kCounterByteIndex] = static_cast<std::uint8_t>(
    (data[protocol::kCounterByteIndex] & protocol::kCounterReservedMask) | high_nibble);
  data[protocol::kCrcByteIndex] = FrameChecksum(data);
}

/// Validates the rolling counter and CRC of an inbound frame.
bool ValidateFrame(const CanFrame & frame)
{
  if (frame.dlc != protocol::kDlc) {
    return false;
  }
  const std::uint8_t expected = FrameChecksum(frame.data);
  return expected == frame.data[protocol::kCrcByteIndex];
}

/// Extracts the rolling counter from byte 6.
std::uint8_t ExtractCounter(const CanFrame & frame)
{
  return static_cast<std::uint8_t>(
    (frame.data[protocol::kCounterByteIndex] >> protocol::kCounterShift) &
    protocol::kCounterMask);
}
}  // namespace

std::uint8_t Crc8SaeJ1850(const std::uint8_t * data, std::size_t length)
{
  // Polynomial 0x1D, initial value 0xFF, final XOR 0xFF, MSB-first.
  constexpr std::uint8_t kPolynomial = 0x1D;
  constexpr std::uint8_t kInitial = 0xFF;
  constexpr std::uint8_t kFinalXor = 0xFF;

  std::uint8_t crc = kInitial;
  if (data == nullptr) {
    return static_cast<std::uint8_t>(crc ^ kFinalXor);
  }

  for (std::size_t i = 0; i < length; ++i) {
    crc = static_cast<std::uint8_t>(crc ^ data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x80U) != 0U) {
        crc = static_cast<std::uint8_t>((crc << 1U) ^ kPolynomial);
      } else {
        crc = static_cast<std::uint8_t>(crc << 1U);
      }
    }
  }
  return static_cast<std::uint8_t>(crc ^ kFinalXor);
}

std::uint8_t FrameChecksum(const std::array<std::uint8_t, 8> & data)
{
  return Crc8SaeJ1850(data.data(), protocol::kCrcByteIndex);
}

std::uint32_t ScaleToUnsigned(double value, double scale, std::uint32_t max_count)
{
  if (!std::isfinite(value) || value <= 0.0 || scale <= 0.0) {
    return 0U;
  }
  const double counts = value / scale;
  if (counts >= static_cast<double>(max_count)) {
    return max_count;
  }
  // Round half away from zero so the encoder is symmetric with the decoder.
  return static_cast<std::uint32_t>(std::llround(counts));
}

std::int32_t ScaleToSigned(double value, double scale)
{
  if (!std::isfinite(value) || scale <= 0.0) {
    return 0;
  }
  const double counts = std::llround(value / scale);
  const double clamped = std::clamp(counts, -32768.0, 32767.0);
  return static_cast<std::int32_t>(clamped);
}

CanFrame EncodeBrakeCommand(const BrakeCommand & command, std::uint8_t counter)
{
  CanFrame frame;
  frame.can_id = protocol::kBrakeCommandId;
  frame.dlc = protocol::kDlc;
  frame.data.fill(0U);

  const std::uint32_t deceleration_counts = ScaleToUnsigned(
    std::min(command.requested_deceleration, protocol::kDecelerationMax),
    protocol::kDecelerationScale, protocol::kDecelerationMaxCount);
  StoreUint16(frame.data, 0, static_cast<std::uint16_t>(deceleration_counts));

  const std::uint32_t pressure_counts = ScaleToUnsigned(
    std::clamp(command.brake_pressure, kMinPressure, kMaxPressure),
    protocol::kPressureScale, protocol::kPressureMaxCount);
  frame.data[2] = static_cast<std::uint8_t>(pressure_counts);

  std::uint8_t status = 0U;
  status = static_cast<std::uint8_t>(status |
    ((command.state << protocol::kStatusStateShift) & protocol::kStatusStateMask));
  if (command.state_changed) {
    status = static_cast<std::uint8_t>(status | protocol::kStatusChangedBit);
  }
  if (command.overridden) {
    status = static_cast<std::uint8_t>(status | protocol::kStatusOverrideBit);
  }
  if (command.valid) {
    status = static_cast<std::uint8_t>(status | protocol::kStatusValidBit);
  }
  frame.data[3] = static_cast<std::uint8_t>(status & static_cast<std::uint8_t>(
    ~protocol::kStatusReservedMask));

  StoreUint16(frame.data, 4, command.target_track_id);

  FinaliseFrame(frame.data, counter);
  return frame;
}

bool DecodeBrakeCommand(const CanFrame & frame, BrakeCommand & command)
{
  if (frame.can_id != protocol::kBrakeCommandId || !ValidateFrame(frame)) {
    return false;
  }

  BrakeCommand decoded;
  decoded.requested_deceleration = static_cast<double>(LoadUint16(frame.data, 0)) *
    protocol::kDecelerationScale;
  decoded.brake_pressure = std::min(
    1.0, static_cast<double>(frame.data[2]) * protocol::kPressureScale);

  const std::uint8_t status = frame.data[3];
  decoded.state = static_cast<std::uint8_t>(
    (status & protocol::kStatusStateMask) >> protocol::kStatusStateShift);
  decoded.state_changed = (status & protocol::kStatusChangedBit) != 0U;
  decoded.overridden = (status & protocol::kStatusOverrideBit) != 0U;
  decoded.valid = (status & protocol::kStatusValidBit) != 0U;
  decoded.target_track_id = LoadUint16(frame.data, 4);

  command = decoded;
  return true;
}

CanFrame EncodeTargetSpeed(const TargetSpeedCommand & command, std::uint8_t counter)
{
  CanFrame frame;
  frame.can_id = protocol::kTargetSpeedId;
  frame.dlc = protocol::kDlc;
  frame.data.fill(0U);

  const std::int32_t target = ScaleToSigned(
    std::clamp(command.target_speed, -protocol::kSpeedMax, protocol::kSpeedMax),
    protocol::kSpeedScale);
  const std::int32_t ego = ScaleToSigned(
    std::clamp(command.ego_speed, -protocol::kSpeedMax, protocol::kSpeedMax),
    protocol::kSpeedScale);

  StoreUint16(frame.data, 0, static_cast<std::uint16_t>(target & 0xFFFF));
  StoreUint16(frame.data, 2, static_cast<std::uint16_t>(ego & 0xFFFF));

  frame.data[4] = static_cast<std::uint8_t>(ScaleToUnsigned(
    command.deceleration, protocol::kCoarseDecelerationScale, 255U));
  frame.data[5] = static_cast<std::uint8_t>(ScaleToUnsigned(
    std::clamp(command.brake_pressure, kMinPressure, kMaxPressure),
    protocol::kPressureScale, protocol::kPressureMaxCount));

  FinaliseFrame(frame.data, counter);
  return frame;
}

bool DecodeTargetSpeed(const CanFrame & frame, TargetSpeedCommand & command)
{
  if (frame.can_id != protocol::kTargetSpeedId || !ValidateFrame(frame)) {
    return false;
  }

  TargetSpeedCommand decoded;
  // Sign-extend the 16-bit little-endian fields.
  decoded.target_speed = static_cast<double>(static_cast<std::int16_t>(LoadUint16(frame.data, 0))) *
    protocol::kSpeedScale;
  decoded.ego_speed = static_cast<double>(static_cast<std::int16_t>(LoadUint16(frame.data, 2))) *
    protocol::kSpeedScale;
  decoded.deceleration = static_cast<double>(frame.data[4]) *
    protocol::kCoarseDecelerationScale;
  decoded.brake_pressure = std::min(
    1.0, static_cast<double>(frame.data[5]) * protocol::kPressureScale);

  command = decoded;
  return true;
}

std::uint8_t FrameCounter(const CanFrame & frame)
{
  return ExtractCounter(frame);
}

}  // namespace sil_can_gateway
