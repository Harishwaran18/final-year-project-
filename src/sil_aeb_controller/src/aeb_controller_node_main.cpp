// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sil_aeb_controller/aeb_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.automatically_declare_parameters_from_overrides(false);

  auto node = std::make_shared<sil_aeb_controller::AebControllerNode>(options);

  // Multi-threaded: the FSM timer must keep running at 50 Hz even while the
  // obstacle and service callbacks are executing.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
