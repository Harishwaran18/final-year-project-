// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_CAN_GATEWAY__CAN_FRAME_CODEC_HPP_
#define SIL_CAN_GATEWAY__CAN_FRAME_CODEC_HPP_

#include <array>
#include <cstdint>

namespace sil_can_gateway
{

/// A classic CAN frame: 11-bit identifier and up to 8 payload bytes.
struct CanFrame
{
  static constexpr std::size_t kMaxDataLength = 8;

  std::uint32_t can_id{0};
  std::uint8_t dlc{0};
  std::array<std::uint8_t, kMaxDataLength> data{};

  bool operator==(const CanFrame & other) const
  {
    return can_id == other.can_id && dlc == other.dlc && data == other.data;
  }
};

/// Signal layout of the SIL AEB CAN protocol.
///
/// Two classic-CAN frames carry the whole brake interface:
///
///   @c kBrakeCommandId (0x1A0)
///     byte 0..1  requested deceleration, unsigned 16-bit little endian,
///                scaling @c kDecelerationScale m/s^2 per count
///     byte 2     brake pressure, unsigned 8-bit, scaling @c kPressureScale
///                per count (saturating at @c kPressureMaxCount)
///     byte 3     status bits: [1:0] AEB state, [2] state_changed,
///                [3] override active, [4] command valid, [7:5] reserved
///     byte 4..5  target track id, unsigned 16-bit little endian
///     byte 6     rolling counter in the high nibble, low nibble reserved
///     byte 7     CRC-8/SAE-J1850 over bytes 0..6
///
///   @c kTargetSpeedId (0x1A1)
///     byte 0..1  target vehicle speed, signed 16-bit little endian,
///                scaling @c kSpeedScale m/s per count
///     byte 2..3  ego speed, signed 16-bit little endian, same scaling
///     byte 4     deceleration request, unsigned 8-bit, 0.01 m/s^2 per count
///     byte 5     brake pressure, unsigned 8-bit, same scaling as above
///     byte 6     rolling counter in the high nibble, low nibble reserved
///     byte 7     CRC-8/SAE-J1850 over bytes 0..6
namespace protocol
{
/// Identifier of the AEB brake command frame.
constexpr std::uint32_t kBrakeCommandId = 0x1A0;
/// Identifier of the target-speed frame.
constexpr std::uint32_t kTargetSpeedId = 0x1A1;
/// Payload length of both frames.
constexpr std::uint8_t kDlc = 8;

/// Deceleration resolution: 0.001 m/s^2 per count.
constexpr double kDecelerationScale = 0.001;
/// Maximum representable deceleration request [m/s^2]; 65535 counts at
/// @ref kDecelerationScale.
constexpr double kDecelerationMax = 65.535;
/// Highest deceleration count; matches @ref kDecelerationMax.
constexpr std::uint32_t kDecelerationMaxCount = 65535U;

/// Speed resolution: 0.001 m/s per count.
constexpr double kSpeedScale = 0.001;
/// Maximum representable speed magnitude [m/s].
constexpr double kSpeedMax = 32.767;

/// Brake pressure resolution: 0.4% per count.
constexpr double kPressureScale = 0.004;
/// Highest pressure count; 250 counts == 100% brake pressure.
constexpr std::uint8_t kPressureMaxCount = 250;
/// Deceleration resolution in the target-speed frame: 0.01 m/s^2 per count.
constexpr double kCoarseDecelerationScale = 0.01;

/// Bit positions inside the brake-command status byte (byte 3).
constexpr std::uint8_t kStatusStateMask = 0x03;      // bits [1:0]
constexpr std::uint8_t kStatusStateShift = 0;
constexpr std::uint8_t kStatusChangedBit = 0x04;    // bit 2
constexpr std::uint8_t kStatusOverrideBit = 0x08;   // bit 3
constexpr std::uint8_t kStatusValidBit = 0x10;      // bit 4
constexpr std::uint8_t kStatusReservedMask = 0xE0;  // bits [7:5]

/// Rolling counter lives in the high nibble of byte 6.
constexpr std::uint8_t kCounterShift = 4;
constexpr std::uint8_t kCounterMask = 0x0F;
constexpr std::uint8_t kCounterReservedMask = 0x0F;

/// Byte index of the rolling-counter/reserved byte.
constexpr std::size_t kCounterByteIndex = 6;
/// Byte index of the CRC byte.
constexpr std::size_t kCrcByteIndex = 7;
}  // namespace protocol

/// Decoded brake command frame.
struct BrakeCommand
{
  double requested_deceleration{0.0};  ///< [m/s^2], >= 0
  double brake_pressure{0.0};          ///< normalised [0, 1]
  std::uint8_t state{0};               ///< AEB FSM state, 0..3
  std::uint16_t target_track_id{0};
  bool state_changed{false};
  bool overridden{false};
  bool valid{false};
};

/// Decoded target-speed frame.
struct TargetSpeedCommand
{
  double target_speed{0.0};        ///< [m/s], signed
  double ego_speed{0.0};           ///< [m/s], signed
  double deceleration{0.0};        ///< [m/s^2], >= 0
  double brake_pressure{0.0};      ///< normalised [0, 1]
};

/// CRC-8/SAE-J1850: polynomial 0x1D, initial value 0xFF, final XOR 0xFF.
///
/// Known-answer vector: the check string "123456789" yields 0x4B.
std::uint8_t Crc8SaeJ1850(const std::uint8_t * data, std::size_t length);

/// Computes the CRC byte for a frame payload over bytes 0..6.
std::uint8_t FrameChecksum(const std::array<std::uint8_t, 8> & data);

/// Scales @p value by @p scale, rounds to nearest and saturates to [0, max].
std::uint32_t ScaleToUnsigned(double value, double scale, std::uint32_t max_count);

/// Scales @p value by @p scale, rounds to nearest and saturates to int16 range.
std::int32_t ScaleToSigned(double value, double scale);

/// Packs a brake command into an 8-byte frame.
///
/// @param command decoded brake command
/// @param counter rolling counter, only the low nibble is used
/// @return the fully populated frame with id @c protocol::kBrakeCommandId
CanFrame EncodeBrakeCommand(const BrakeCommand & command, std::uint8_t counter);

/// Unpacks a brake command frame.
///
/// @param frame  frame to decode; the id and DLC are validated
/// @param[out] command decoded command, untouched on failure
/// @return false when the frame is malformed or the CRC does not match
bool DecodeBrakeCommand(const CanFrame & frame, BrakeCommand & command);

/// Packs a target-speed command into an 8-byte frame.
CanFrame EncodeTargetSpeed(const TargetSpeedCommand & command, std::uint8_t counter);

/// Unpacks a target-speed frame.
bool DecodeTargetSpeed(const CanFrame & frame, TargetSpeedCommand & command);

/// Extracts the rolling counter nibble from a frame (byte 6, high nibble).
std::uint8_t FrameCounter(const CanFrame & frame);

}  // namespace sil_can_gateway

#endif  // SIL_CAN_GATEWAY__CAN_FRAME_CODEC_HPP_
