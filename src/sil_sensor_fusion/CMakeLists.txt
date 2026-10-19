cmake_minimum_required(VERSION 3.16)
project(sil_sensor_fusion VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sil_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search segmentation)

# ---------------------------------------------------------------------------
# Pure algorithm library: Eigen3 + PCL only, no ROS types. This is the unit
# under test and the library reused by offline SIL replay tooling.
# ---------------------------------------------------------------------------
add_library(sil_sensor_fusion_core STATIC
  src/ekf_tracker.cpp
  src/cluster_geometry.cpp
  src/multi_target_tracker.cpp
  src/camera_projection.cpp
  src/lidar_clusterer.cpp
)
target_include_directories(sil_sensor_fusion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(sil_sensor_fusion_core PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(sil_sensor_fusion_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

# ---------------------------------------------------------------------------
# ROS 2 node.
# ---------------------------------------------------------------------------
add_library(sil_sensor_fusion_node_lib STATIC src/sensor_fusion_node.cpp)
target_include_directories(sil_sensor_fusion_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(sil_sensor_fusion_node_lib sil_sensor_fusion_core)
ament_target_dependencies(sil_sensor_fusion_node_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  sil_interfaces
)

add_executable(sil_sensor_fusion_node src/sensor_fusion_node_main.cpp)
target_link_libraries(sil_sensor_fusion_node sil_sensor_fusion_node_lib)

# ---------------------------------------------------------------------------
# Install rules.
# ---------------------------------------------------------------------------
install(TARGETS
  sil_sensor_fusion_core
  sil_sensor_fusion_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sil_sensor_fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Tests.
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure-math suites: runnable without a ROS graph or a CAN interface.
  ament_add_gtest(test_ekf_tracker test/test_ekf_tracker.cpp)
  target_link_libraries(test_ekf_tracker sil_sensor_fusion_core)

  ament_add_gtest(test_multi_target_tracker test/test_multi_target_tracker.cpp)
  target_link_libraries(test_multi_target_tracker sil_sensor_fusion_core)

  ament_add_gtest(test_cluster_geometry test/test_cluster_geometry.cpp)
  target_link_libraries(test_cluster_geometry sil_sensor_fusion_core)

  ament_add_gtest(test_lidar_clusterer test/test_lidar_clusterer.cpp)
  target_link_libraries(test_lidar_clusterer sil_sensor_fusion_core)

  ament_add_gtest(test_camera_projection test/test_camera_projection.cpp)
  target_link_libraries(test_camera_projection sil_sensor_fusion_core)

  # Node-level smoke test: catches parameter declaration and construction bugs.
  ament_add_gtest(test_sensor_fusion_node test/test_sensor_fusion_node.cpp)
  target_link_libraries(test_sensor_fusion_node sil_sensor_fusion_node_lib)

  # Latency budget: the fusion step must stay under 18 ms.
  ament_add_gtest(test_fusion_latency test/test_fusion_latency.cpp)
  target_link_libraries(test_fusion_latency sil_sensor_fusion_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(sil_sensor_fusion_core sil_sensor_fusion_node_lib)
ament_export_dependencies(Eigen3 PCL rclcpp sil_interfaces)

ament_package()
