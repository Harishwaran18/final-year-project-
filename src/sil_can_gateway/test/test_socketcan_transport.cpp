// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cstring>
#include <string>

#include "sil_can_gateway/socketcan_transport.hpp"

namespace sil_can_gateway
{
namespace
{

/// Transport tests that do not need a CAN interface.
///
/// The end-to-end loopback test lives in the SIL harness and requires
/// `modprobe vcan` plus `ip link add dev vcan0 type vcan`; it is skipped here so
/// the suite stays runnable in a plain container. What is checked instead is the
/// failure behaviour, which is what the gateway relies on to degrade to dry-run
/// mode rather than crash.
TEST(SocketCanTransportTest, StartsClosed)
{
  SocketCanTransport transport;
  EXPECT_FALSE(transport.is_open());
  EXPECT_TRUE(transport.interface_name().empty());
  EXPECT_EQ(0U, transport.frames_sent());
  EXPECT_EQ(0U, transport.frames_received());
}

TEST(SocketCanTransportTest, SendOnAClosedTransportFails)
{
  SocketCanTransport transport;
  const CanFrame frame = EncodeBrakeCommand(BrakeCommand{}, 0);

  EXPECT_FALSE(transport.Send(frame));
  EXPECT_FALSE(transport.last_error().empty());
  EXPECT_EQ(0U, transport.frames_sent());
}

TEST(SocketCanTransportTest, ReceiveOnAClosedTransportFails)
{
  SocketCanTransport transport;
  CanFrame frame;

  EXPECT_FALSE(transport.Receive(frame, 0));
  EXPECT_FALSE(transport.last_error().empty());
  EXPECT_EQ(0U, transport.frames_received());
}

TEST(SocketCanTransportTest, OpenFailsForAMissingInterface)
{
  SocketCanTransport transport;

  EXPECT_FALSE(transport.Open("sil_no_such_can0"));
  EXPECT_FALSE(transport.is_open());
  EXPECT_FALSE(transport.last_error().empty())
    << "the failure must be reportable so the node can log it";
}

TEST(SocketCanTransportTest, OpenRejectsAnOverlongInterfaceName)
{
  SocketCanTransport transport;

  EXPECT_FALSE(transport.Open(std::string(64, 'x')));
  EXPECT_FALSE(transport.is_open());
  EXPECT_FALSE(transport.last_error().empty());
}

TEST(SocketCanTransportTest, CloseIsIdempotent)
{
  SocketCanTransport transport;
  transport.Close();
  transport.Close();
  EXPECT_FALSE(transport.is_open());
}

TEST(SocketCanTransportTest, MoveTransfersOwnershipAndLeavesTheSourceClosed)
{
  SocketCanTransport source;
  // Open against a missing interface leaves it closed but with an error set;
  // that is enough to observe the move of the bookkeeping fields.
  source.Open("sil_no_such_can0");

  SocketCanTransport moved(std::move(source));
  EXPECT_FALSE(moved.is_open());
  EXPECT_FALSE(moved.last_error().empty());
  EXPECT_TRUE(source.interface_name().empty());
}

TEST(SocketCanTransportTest, MoveAssignmentReleasesTheExistingSocket)
{
  SocketCanTransport target;
  SocketCanTransport source;
  source.Open("sil_no_such_can0");

  target = std::move(source);
  EXPECT_FALSE(target.is_open());
  EXPECT_FALSE(target.last_error().empty());
}

}  // namespace
}  // namespace sil_can_gateway
