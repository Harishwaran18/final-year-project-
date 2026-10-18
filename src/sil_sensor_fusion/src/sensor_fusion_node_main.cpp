// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "sil_sensor_fusion/sensor_fusion_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(false);
  options.automatically_declare_parameters_from_overrides(false);

  auto node = std::make_shared<sil_sensor_fusion::SensorFusionNode>(options);

  // Single-threaded executor: fusion is deterministic per sweep and the
  // callbacks are cheap, so we avoid the extra locking of a multi-threaded one.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
