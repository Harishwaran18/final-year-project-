#!/usr/bin/env bash
# Builds the workspace and runs every test suite.
#
#   ./scripts/run_tests.sh          # ROS-free core tests (fast)
#   ./scripts/run_tests.sh --ros    # colcon build + colcon test (full stack)

set -euo pipefail

cd "$(dirname "$0")/.."

USE_ROS=0
for arg in "$@"; do
  case "${arg}" in
    --ros) USE_ROS=1 ;;
    -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "unknown argument: ${arg}" >&2; exit 2 ;;
  esac
done

if [[ "${USE_ROS}" -eq 0 ]]; then
  echo "==> Configuring the ROS-free core build"
  cmake -S core -B core/build -DCMAKE_BUILD_TYPE=RelWithDebInfo

  echo "==> Building"
  cmake --build core/build --parallel

  echo "==> Running the GoogleTest suites"
  ctest --test-dir core/build --output-on-failure
  exit 0
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  echo "Run ./scripts/install_dependencies.sh first." >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "==> colcon build"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

echo "==> colcon test"
colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --all --verbose
