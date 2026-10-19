cmake_minimum_required(VERSION 3.16)
project(sil_carla_bridge VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sil_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

# ---------------------------------------------------------------------------
# Pure library: frame maths and LiDAR synthesis, no ROS types.
# ---------------------------------------------------------------------------
add_library(sil_carla_bridge_core STATIC
  src/frame_transform.cpp
  src/lidar_synthesizer.cpp
)
target_include_directories(sil_carla_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(sil_carla_bridge_core PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(sil_carla_bridge_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})

# ---------------------------------------------------------------------------
# ROS 2 nodes.
# ---------------------------------------------------------------------------
add_library(sil_carla_bridge_node_lib STATIC src/carla_bridge_node.cpp)
target_include_directories(sil_carla_bridge_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(sil_carla_bridge_node_lib sil_carla_bridge_core)
ament_target_dependencies(sil_carla_bridge_node_lib
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  sil_interfaces
)

add_executable(sil_carla_bridge_node src/carla_bridge_node_main.cpp)
target_link_libraries(sil_carla_bridge_node sil_carla_bridge_node_lib)

add_executable(carla_scenario_publisher
  src/scenario_publisher.cpp
  src/scenario_publisher_main.cpp
)
target_include_directories(carla_scenario_publisher PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(carla_scenario_publisher
  rclcpp
  sil_interfaces
)

# ---------------------------------------------------------------------------
# Install rules.
# ---------------------------------------------------------------------------
install(TARGETS
  sil_carla_bridge_core
  sil_carla_bridge_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  sil_carla_bridge_node
  carla_scenario_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_transform test/test_frame_transform.cpp)
  target_link_libraries(test_frame_transform sil_carla_bridge_core)

  ament_add_gtest(test_lidar_synthesizer test/test_lidar_synthesizer.cpp)
  target_link_libraries(test_lidar_synthesizer sil_carla_bridge_core)

  ament_add_gtest(test_carla_bridge_node test/test_carla_bridge_node.cpp)
  target_link_libraries(test_carla_bridge_node sil_carla_bridge_node_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(sil_carla_bridge_core sil_carla_bridge_node_lib)
ament_export_dependencies(Eigen3 PCL rclcpp sil_interfaces)

ament_package()
