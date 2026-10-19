cmake_minimum_required(VERSION 3.16)
project(sil_can_gateway VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sil_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Frame codec + SocketCAN transport. No ROS types: the codec is pure bit
# manipulation and the transport is a thin PF_CAN wrapper, so both are unit
# testable on a host without a ROS graph or a CAN interface.
# ---------------------------------------------------------------------------
add_library(sil_can_gateway_core STATIC
  src/can_frame_codec.cpp
  src/socketcan_transport.cpp
)
target_include_directories(sil_can_gateway_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(sil_can_gateway_node_lib STATIC src/can_gateway_node.cpp)
target_include_directories(sil_can_gateway_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sil_can_gateway_node_lib sil_can_gateway_core)
ament_target_dependencies(sil_can_gateway_node_lib
  rclcpp
  std_msgs
  sil_interfaces
)

add_executable(sil_can_gateway_node src/can_gateway_node_main.cpp)
target_link_libraries(sil_can_gateway_node sil_can_gateway_node_lib)

install(TARGETS
  sil_can_gateway_core
  sil_can_gateway_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sil_can_gateway_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_can_frame_codec test/test_can_frame_codec.cpp)
  target_link_libraries(test_can_frame_codec sil_can_gateway_core)

  ament_add_gtest(test_crc8 test/test_crc8.cpp)
  target_link_libraries(test_crc8 sil_can_gateway_core)

  ament_add_gtest(test_scaling test/test_scaling.cpp)
  target_link_libraries(test_scaling sil_can_gateway_core)

  ament_add_gtest(test_socketcan_transport test/test_socketcan_transport.cpp)
  target_link_libraries(test_socketcan_transport sil_can_gateway_core)

  ament_add_gtest(test_can_gateway_node test/test_can_gateway_node.cpp)
  target_link_libraries(test_can_gateway_node sil_can_gateway_node_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(sil_can_gateway_core sil_can_gateway_node_lib)
ament_export_dependencies(rclcpp sil_interfaces)

ament_package()
