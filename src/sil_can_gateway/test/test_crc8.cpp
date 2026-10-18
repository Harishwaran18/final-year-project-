// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "sil_can_gateway/can_frame_codec.hpp"

namespace sil_can_gateway
{
namespace
{

TEST(Crc8Test, MatchesTheSaeJ1850CheckVector)
{
  // The canonical "123456789" check string yields 0x4B for CRC-8/SAE-J1850.
  const std::array<std::uint8_t, 9> check_string{
    '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(0x4B, Crc8SaeJ1850(check_string.data(), check_string.size()));
}

TEST(Crc8Test, EmptyInputReturnsTheXorOutValue)
{
  const std::array<std::uint8_t, 1> dummy{0};
  EXPECT_EQ(0x00, Crc8SaeJ1850(dummy.data(), 0));
}

TEST(Crc8Test, NullPointerIsHandled)
{
  EXPECT_EQ(0x00, Crc8SaeJ1850(nullptr, 5));
}

TEST(Crc8Test, SingleZeroByte)
{
  // 0x00 through the algorithm: crc = 0xFF ^ 0x00 = 0xFF, eight shifts with the
  // polynomial applied, then XOR 0xFF.
  const std::array<std::uint8_t, 1> data{0x00};
  const std::uint8_t crc = Crc8SaeJ1850(data.data(), data.size());
  EXPECT_NE(0x00, crc);
  EXPECT_NE(0xFF, crc);
}

TEST(Crc8Test, DetectsASingleBitFlip)
{
  const std::array<std::uint8_t, 6> original{0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
  std::array<std::uint8_t, 6> corrupted = original;
  corrupted[3] ^= 0x01;

  EXPECT_NE(Crc8SaeJ1850(original.data(), original.size()),
    Crc8SaeJ1850(corrupted.data(), corrupted.size()));
}

TEST(Crc8Test, DetectsEverySingleByteChange)
{
  const std::array<std::uint8_t, 7> original{0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
  const std::uint8_t reference = Crc8SaeJ1850(original.data(), original.size());

  for (std::size_t index = 0; index < original.size(); ++index) {
    for (int bit = 0; bit < 8; ++bit) {
      std::array<std::uint8_t, 7> corrupted = original;
      corrupted[index] = static_cast<std::uint8_t>(corrupted[index] ^ (1U << bit));
      EXPECT_NE(reference, Crc8SaeJ1850(corrupted.data(), corrupted.size()))
        << "byte " << index << " bit " << bit;
    }
  }
}

TEST(Crc8Test, IsDeterministic)
{
  const std::array<std::uint8_t, 8> data{1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_EQ(Crc8SaeJ1850(data.data(), data.size()),
    Crc8SaeJ1850(data.data(), data.size()));
}

TEST(Crc8Test, OrderMatters)
{
  const std::array<std::uint8_t, 3> forward{0x01, 0x02, 0x03};
  const std::array<std::uint8_t, 3> reverse{0x03, 0x02, 0x01};
  EXPECT_NE(Crc8SaeJ1850(forward.data(), forward.size()),
    Crc8SaeJ1850(reverse.data(), reverse.size()));
}

TEST(Crc8Test, FrameChecksumCoversBytesZeroToSix)
{
  std::array<std::uint8_t, 8> data{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x00};

  const std::uint8_t checksum = FrameChecksum(data);
  EXPECT_EQ(Crc8SaeJ1850(data.data(), 7), checksum);

  // Byte 7 is the CRC slot and must not influence the checksum it holds.
  data[7] = 0xFF;
  EXPECT_EQ(checksum, FrameChecksum(data));
}

TEST(Crc8Test, FrameChecksumChangesWithAnyPayloadByte)
{
  std::array<std::uint8_t, 8> data{0, 0, 0, 0, 0, 0, 0, 0};
  const std::uint8_t reference = FrameChecksum(data);

  for (std::size_t index = 0; index < 7; ++index) {
    std::array<std::uint8_t, 8> modified = data;
    modified[index] = 0x01;
    EXPECT_NE(reference, FrameChecksum(modified)) << "byte " << index;
  }
}

}  // namespace
}  // namespace sil_can_gateway
