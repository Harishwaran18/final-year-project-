# Standalone (ROS-free) build of the SIL AEB algorithm cores plus their
# GoogleTest suites. Included by the top-level CMakeLists.txt; it must not call
# project() itself.
#
#   cmake -S . -B build
#   cmake --build build -j
#   ctest --test-dir build --output-on-failure

find_package(Eigen3 3.3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search segmentation)
find_package(GTest REQUIRED)
include(GoogleTest)
include(CTest)

# ---------------------------------------------------------------------------
# Core libraries, built directly from source so this project needs no ROS.
# ---------------------------------------------------------------------------
add_library(sil_aeb_core STATIC
  ../src/sil_sensor_fusion/src/ekf_tracker.cpp
  ../src/sil_sensor_fusion/src/cluster_geometry.cpp
  ../src/sil_sensor_fusion/src/multi_target_tracker.cpp
  ../src/sil_sensor_fusion/src/camera_projection.cpp
  ../src/sil_sensor_fusion/src/lidar_clusterer.cpp
  ../src/sil_aeb_controller/src/aeb_fsm.cpp
  ../src/sil_can_gateway/src/can_frame_codec.cpp
  ../src/sil_carla_bridge/src/frame_transform.cpp
  ../src/sil_carla_bridge/src/lidar_synthesizer.cpp
)
target_include_directories(sil_aeb_core PUBLIC
  ../src/sil_sensor_fusion/include
  ../src/sil_aeb_controller/include
  ../src/sil_can_gateway/include
  ../src/sil_carla_bridge/include
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(sil_aeb_core PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(sil_aeb_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

# ---------------------------------------------------------------------------
# Test suites. Each test lives next to the package it covers; the paths here
# mirror the ament_add_gtest() calls in the package CMakeLists.txt files.
# ---------------------------------------------------------------------------
function(sil_add_core_test name path)
  add_executable(${name} ${path})
  target_link_libraries(${name} PRIVATE sil_aeb_core GTest::gtest GTest::gtest_main)
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

set(sil_fusion_test_dir ../src/sil_sensor_fusion/test)
set(sil_aeb_test_dir ../src/sil_aeb_controller/test)
set(sil_can_test_dir ../src/sil_can_gateway/test)
set(sil_bridge_test_dir ../src/sil_carla_bridge/test)

sil_add_core_test(test_ekf_tracker ${sil_fusion_test_dir}/test_ekf_tracker.cpp)
sil_add_core_test(test_multi_target_tracker ${sil_fusion_test_dir}/test_multi_target_tracker.cpp)
sil_add_core_test(test_cluster_geometry ${sil_fusion_test_dir}/test_cluster_geometry.cpp)
sil_add_core_test(test_lidar_clusterer ${sil_fusion_test_dir}/test_lidar_clusterer.cpp)
sil_add_core_test(test_camera_projection ${sil_fusion_test_dir}/test_camera_projection.cpp)
sil_add_core_test(test_fusion_latency ${sil_fusion_test_dir}/test_fusion_latency.cpp)

sil_add_core_test(test_ttc_calculator ${sil_aeb_test_dir}/test_ttc_calculator.cpp)
sil_add_core_test(test_aeb_fsm ${sil_aeb_test_dir}/test_aeb_fsm.cpp)
sil_add_core_test(test_target_selector ${sil_aeb_test_dir}/test_target_selector.cpp)
sil_add_core_test(test_aeb_scenario ${sil_aeb_test_dir}/test_aeb_scenario.cpp)

sil_add_core_test(test_can_frame_codec ${sil_can_test_dir}/test_can_frame_codec.cpp)
sil_add_core_test(test_crc8 ${sil_can_test_dir}/test_crc8.cpp)
sil_add_core_test(test_scaling ${sil_can_test_dir}/test_scaling.cpp)

sil_add_core_test(test_frame_transform ${sil_bridge_test_dir}/test_frame_transform.cpp)
sil_add_core_test(test_lidar_synthesizer ${sil_bridge_test_dir}/test_lidar_synthesizer.cpp)
