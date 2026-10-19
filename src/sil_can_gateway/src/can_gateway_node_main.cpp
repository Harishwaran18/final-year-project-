// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sil_can_gateway/can_gateway_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.automatically_declare_parameters_from_overrides(false);

  auto node = std::make_shared<sil_can_gateway::CanGatewayNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
