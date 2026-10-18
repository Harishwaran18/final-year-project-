# AGENTS.md

Repository-specific knowledge for the SIL AEB stack.

## What this is

A ROS 2 Humble workspace (C++17) that turns LiDAR/camera input into an AEB
braking command on a virtual CAN bus. Six packages under `src/` plus a ROS-free
core harness under `core/`.

## Build and test

```bash
# ROS-free core (fast, no ROS needed). ~2 s, 245 tests.
./scripts/run_tests.sh
# equivalently:
cmake -S core -B core/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build core/build --parallel
ctest --test-dir core/build --output-on-failure

# Full ROS 2 workspace. ~2.5 min cold, 315 tests.
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+
colcon test-result --all
```

## Layout notes that are easy to get wrong

- `core/CMakeLists.txt` is **not** a ROS 2 package. `core/COLCON_IGNORE` stops
  colcon from discovering it. Do not remove that marker: without it, colcon
  treats the ROS-free harness as a package named after the root directory and
  the build fails on `src/` paths.
- Core test paths in `core/cmake/core_tests.cmake` are relative to `core/`, so
  they read `../src/<pkg>/...`.
- CI runs `cmake -S core -B build`, not `-S .`.

## Package conventions

Each package separates a ROS-free numerical core from its ROS wrapper:

- `<pkg>_core` — static library, no ROS headers, unit tested by the pure suites.
- `<pkg>_node_lib` — links `_core`, adds rclcpp subscriptions/publishers.
- `<pkg>_node` — thin `main()` that spins the node.

`sil_interfaces` is the exception: it is a pure rosidl package.

## CMake gotchas

- `ament_target_dependencies` uses the **plain** `target_link_libraries`
  signature. Never mix it with `PUBLIC`/`PRIVATE` on the same target; CMake
  errors with "all uses ... must be either all-keyword or all-plain".
- `sil_interfaces` must **not** declare `LANGUAGES CXX` in `project()`: rosidl
  generates C and Python targets and CMake then cannot determine their link
  language.
- Use `#include <Eigen/Geometry>` explicitly wherever `cross()` is called.
  `Eigen/Core` alone leaves `cross_product_return_type` undefined and the link
  fails (seen with PCL 1.12 / Eigen 3.4).

## AEB state encoding is frozen

`AebState` values (`kNormal=0`, `kForwardCollisionWarning=1`,
`kPartialBraking=2`, `kFullEmergencyBrake=3`) are part of the published
`SafetyAlert` message and the CAN status bits. Never reorder them.

## Test isolation requirement

Node tests create real rclcpp nodes that publish on absolute-looking topic
names (`aeb/alert`, `vehicle/ego_speed`, ...). `colcon test` runs test binaries
concurrently in one ROS domain, so two binaries can see each other's messages.
Every node test fixture must therefore give each test case its own namespace via
a `Isolate(NodeOptions)` helper that passes `--ros-args -r __ns:=<per-test>`.
Without this you get flaky cross-talk failures that pass when run alone.

## CAN protocol

Two classic 8-byte frames, both CRC-8/SAE-J1850 over bytes 0..6:

- `0x1A0` brake command: decel (`uint16`, 0.001 m/s²), pressure, status bits,
  target track id, rolling counter.
- `0x1A1` target speed: target and ego speed (`int16`, 0.001 m/s), decel
  (`uint8`, 0.01 m/s²), pressure, counter.

Decode validates id, DLC, and CRC, and saturates rather than wrapping.

## Docker-based verification

The host has no ROS installation. To verify the ROS build, run the official
image against the workspace:

```bash
sudo docker run --rm -v "$PWD:/ws:rw" ros:humble-ros-base bash -c '
  apt-get update -qq &&
  apt-get install -y -qq --no-install-recommends \
    python3-colcon-common-extensions ros-humble-ament-cmake-gtest \
    ros-humble-pcl-conversions ros-humble-rosidl-default-generators \
    libpcl-dev libeigen3-dev libopencv-dev can-utils iproute2 &&
  source /opt/ros/humble/setup.bash && cd /ws &&
  colcon build --symlink-install && colcon test'
```

Notes:
- Run it in the background with output to a log and poll; a cold run exceeds the
  foreground command timeout.
- `libpcl-dev` is required for the *runtime* too: without it the fusion test
  binaries fail with `libpcl_segmentation.so.1.12: cannot open shared object
  file` even though the build succeeded.
- Build artefacts land root-owned in the workspace; clean with `sudo rm -rf
  build install log`.
