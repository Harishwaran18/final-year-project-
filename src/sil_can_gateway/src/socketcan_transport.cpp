// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "sil_can_gateway/socketcan_transport.hpp"

#include <cerrno>
#include <cstring>
#include <string>
#include <utility>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace sil_can_gateway
{
namespace
{
/// Human readable errno text for the last failure.
std::string ErrnoMessage(const char * operation)
{
  return std::string(operation) + " failed: " + std::strerror(errno);
}

/// Copies our POD frame into the kernel's struct can_frame.
void ToKernelFrame(const CanFrame & source, struct can_frame & target)
{
  std::memset(&target, 0, sizeof(target));
  target.can_id = source.can_id & CAN_SFF_MASK;
  target.can_dlc = source.dlc;
  for (std::size_t i = 0; i < CanFrame::kMaxDataLength; ++i) {
    target.data[i] = source.data[i];
  }
}

/// Copies a kernel frame into our POD representation.
void FromKernelFrame(const struct can_frame & source, CanFrame & target)
{
  target.can_id = source.can_id & CAN_SFF_MASK;
  target.dlc = source.can_dlc > CanFrame::kMaxDataLength ?
    CanFrame::kMaxDataLength : source.can_dlc;
  target.data.fill(0U);
  for (std::size_t i = 0; i < target.dlc; ++i) {
    target.data[i] = source.data[i];
  }
}
}  // namespace

SocketCanTransport::~SocketCanTransport()
{
  Close();
}

SocketCanTransport::SocketCanTransport(SocketCanTransport && other) noexcept
: socket_fd_(other.socket_fd_),
  interface_name_(std::move(other.interface_name_)),
  last_error_(std::move(other.last_error_)),
  frames_sent_(other.frames_sent_),
  frames_received_(other.frames_received_)
{
  other.socket_fd_ = -1;
  other.frames_sent_ = 0;
  other.frames_received_ = 0;
}

SocketCanTransport & SocketCanTransport::operator=(SocketCanTransport && other) noexcept
{
  if (this != &other) {
    Close();
    socket_fd_ = other.socket_fd_;
    interface_name_ = std::move(other.interface_name_);
    last_error_ = std::move(other.last_error_);
    frames_sent_ = other.frames_sent_;
    frames_received_ = other.frames_received_;
    other.socket_fd_ = -1;
    other.frames_sent_ = 0;
    other.frames_received_ = 0;
  }
  return *this;
}

bool SocketCanTransport::Open(const std::string & interface_name, bool loopback)
{
  Close();
  last_error_.clear();

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    last_error_ = ErrnoMessage("socket(PF_CAN)");
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  // ifr_name is IFNAMSIZ bytes including the terminator.
  if (interface_name.size() >= IFNAMSIZ) {
    last_error_ = "interface name too long: " + interface_name;
    ::close(fd);
    return false;
  }
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);

  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    last_error_ = ErrnoMessage(("ioctl(SIOCGIFINDEX) on " + interface_name).c_str());
    ::close(fd);
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    last_error_ = ErrnoMessage(("bind(" + interface_name + ")").c_str());
    ::close(fd);
    return false;
  }

  const int loopback_value = loopback ? 1 : 0;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback_value,
    sizeof(loopback_value)) < 0)
  {
    last_error_ = ErrnoMessage("setsockopt(CAN_RAW_LOOPBACK)");
    ::close(fd);
    return false;
  }

  socket_fd_ = fd;
  interface_name_ = interface_name;
  frames_sent_ = 0;
  frames_received_ = 0;
  return true;
}

void SocketCanTransport::Close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  interface_name_.clear();
}

bool SocketCanTransport::Send(const CanFrame & frame)
{
  if (socket_fd_ < 0) {
    last_error_ = "send on a closed SocketCAN transport";
    return false;
  }

  struct can_frame kernel_frame;
  ToKernelFrame(frame, kernel_frame);

  const ssize_t written = ::write(
    socket_fd_, &kernel_frame, sizeof(kernel_frame));
  if (written != static_cast<ssize_t>(sizeof(kernel_frame))) {
    last_error_ = ErrnoMessage("write(can_frame)");
    return false;
  }

  ++frames_sent_;
  last_error_.clear();
  return true;
}

bool SocketCanTransport::Receive(CanFrame & frame, int timeout_ms)
{
  if (socket_fd_ < 0) {
    last_error_ = "receive on a closed SocketCAN transport";
    return false;
  }

  if (timeout_ms >= 0) {
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    if (::setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
      last_error_ = ErrnoMessage("setsockopt(SO_RCVTIMEO)");
      return false;
    }
  }

  struct can_frame kernel_frame;
  const ssize_t bytes = ::read(socket_fd_, &kernel_frame, sizeof(kernel_frame));
  if (bytes != static_cast<ssize_t>(sizeof(kernel_frame))) {
    last_error_ = bytes < 0 ? ErrnoMessage("read(can_frame)") : "short CAN read";
    return false;
  }

  FromKernelFrame(kernel_frame, frame);
  ++frames_received_;
  last_error_.clear();
  return true;
}

}  // namespace sil_can_gateway
