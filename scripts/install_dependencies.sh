#!/usr/bin/env bash
# Installs the dependencies needed to build and run the SIL AEB stack.
#
#   ./scripts/install_dependencies.sh          # ROS 2 Humble + sensor/CAN stack
#   ./scripts/install_dependencies.sh --core   # ROS-free core build only
#
# Ubuntu 22.04 (Jammy) is assumed, which is the platform ROS 2 Humble targets.

set -euo pipefail

CORE_ONLY=0
for arg in "$@"; do
  case "${arg}" in
    --core) CORE_ONLY=1 ;;
    -h|--help)
      sed -n '2,8p' "$0"
      exit 0
      ;;
    *)
      echo "unknown argument: ${arg}" >&2
      exit 2
      ;;
  esac
done

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

echo "==> Updating the package index"
${SUDO} apt-get update

echo "==> Installing build tooling and the numerical/vision dependencies"
${SUDO} apt-get install -y --no-install-recommends \
  build-essential cmake git \
  libeigen3-dev libpcl-dev libopencv-dev \
  libgtest-dev

echo "==> Installing SocketCAN tooling"
${SUDO} apt-get install -y --no-install-recommends \
  can-utils iproute2 kmod

if [[ "${CORE_ONLY}" -eq 1 ]]; then
  echo "==> Core-only install complete; skipping ROS 2."
  exit 0
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "==> Adding the ROS 2 apt repository"
  ${SUDO} apt-get install -y --no-install-recommends \
    software-properties-common curl gnupg lsb-release

  ${SUDO} add-apt-repository -y universe
  ${SUDO} curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
    | ${SUDO} tee /etc/apt/sources.list.d/ros2.list >/dev/null
  ${SUDO} apt-get update

  echo "==> Installing ROS 2 Humble"
  ${SUDO} apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-pcl-conversions \
    ros-humble-rosidl-default-generators \
    ros-humble-launch-testing-ament-cmake
fi

echo "==> Initialising rosdep (if needed)"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  ${SUDO} rosdep init || true
fi
rosdep update --rosdistro humble || true

echo
echo "Done. Source the workspace with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --symlink-install"
