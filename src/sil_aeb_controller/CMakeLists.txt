cmake_minimum_required(VERSION 3.16)
project(sil_aeb_controller VERSION 0.1.0 LANGUAGES CXX)

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
# Pure control library: no ROS types, fully unit testable.
# ---------------------------------------------------------------------------
add_library(sil_aeb_controller_core STATIC
  src/aeb_fsm.cpp
)
target_include_directories(sil_aeb_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# ROS 2 node.
# ---------------------------------------------------------------------------
add_library(sil_aeb_controller_node_lib STATIC src/aeb_controller_node.cpp)
target_include_directories(sil_aeb_controller_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sil_aeb_controller_node_lib sil_aeb_controller_core)
ament_target_dependencies(sil_aeb_controller_node_lib
  rclcpp
  std_msgs
  sil_interfaces
)

add_executable(sil_aeb_controller_node src/aeb_controller_node_main.cpp)
target_link_libraries(sil_aeb_controller_node sil_aeb_controller_node_lib)

install(TARGETS
  sil_aeb_controller_core
  sil_aeb_controller_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sil_aeb_controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_ttc_calculator test/test_ttc_calculator.cpp)
  target_link_libraries(test_ttc_calculator sil_aeb_controller_core)

  ament_add_gtest(test_aeb_fsm test/test_aeb_fsm.cpp)
  target_link_libraries(test_aeb_fsm sil_aeb_controller_core)

  ament_add_gtest(test_target_selector test/test_target_selector.cpp)
  target_link_libraries(test_target_selector sil_aeb_controller_core)

  ament_add_gtest(test_aeb_scenario test/test_aeb_scenario.cpp)
  target_link_libraries(test_aeb_scenario sil_aeb_controller_core)

  ament_add_gtest(test_aeb_controller_node test/test_aeb_controller_node.cpp)
  target_link_libraries(test_aeb_controller_node sil_aeb_controller_node_lib)
endif()

ament_export_include_directories(include)
ament_export_libraries(sil_aeb_controller_core sil_aeb_controller_node_lib)
ament_export_dependencies(rclcpp sil_interfaces)

ament_package()
