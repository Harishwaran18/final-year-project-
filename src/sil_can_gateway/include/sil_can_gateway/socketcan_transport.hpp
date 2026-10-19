// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef SIL_CAN_GATEWAY__SOCKETCAN_TRANSPORT_HPP_
#define SIL_CAN_GATEWAY__SOCKETCAN_TRANSPORT_HPP_

#include <string>

#include "sil_can_gateway/can_frame_codec.hpp"

namespace sil_can_gateway
{

/// Raw Linux SocketCAN transport for a single interface (typically @c vcan0).
///
/// Thin RAII wrapper over the PF_CAN raw socket: open the interface, write
/// classic 8-byte frames, read them back for SIL verification, then close.
/// The class deliberately performs no framing logic; that lives in
/// @ref can_frame_codec.hpp so it stays unit testable without a CAN interface.
class SocketCanTransport
{
public:
  SocketCanTransport() = default;
  ~SocketCanTransport();

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;
  SocketCanTransport(SocketCanTransport && other) noexcept;
  SocketCanTransport & operator=(SocketCanTransport && other) noexcept;

  /// Binds a raw CAN socket to @p interface_name.
  ///
  /// @param interface_name interface such as @c vcan0; it must already exist
  /// @param loopback when true the kernel echoes written frames back to this
  ///        socket, which is what the SIL loopback test relies on
  /// @return false when the socket cannot be created or bound
  bool Open(const std::string & interface_name, bool loopback = true);

  /// Closes the socket; safe to call repeatedly.
  void Close();

  bool is_open() const {return socket_fd_ >= 0;}

  /// The bound interface name, or an empty string when closed.
  const std::string & interface_name() const {return interface_name_;}

  /// Sends @p frame as a classic CAN frame.
  ///
  /// @return false on a short write or a socket error; @ref last_error() then
  ///         holds the errno-derived description
  bool Send(const CanFrame & frame);

  /// Receives one frame, waiting up to @p timeout_ms.
  ///
  /// @param[out] frame received frame, untouched on failure
  /// @param timeout_ms 0 polls without blocking, negative blocks indefinitely
  /// @return false on timeout, a short read, or a socket error
  bool Receive(CanFrame & frame, int timeout_ms = 100);

  /// Description of the most recent failure; empty after a success.
  const std::string & last_error() const {return last_error_;}

  /// Number of frames successfully written since Open().
  std::uint64_t frames_sent() const {return frames_sent_;}

  /// Number of frames successfully read since Open().
  std::uint64_t frames_received() const {return frames_received_;}

private:
  int socket_fd_{-1};
  std::string interface_name_;
  std::string last_error_;
  std::uint64_t frames_sent_{0};
  std::uint64_t frames_received_{0};
};

}  // namespace sil_can_gateway

#endif  // SIL_CAN_GATEWAY__SOCKETCAN_TRANSPORT_HPP_
