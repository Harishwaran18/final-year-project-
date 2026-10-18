// Copyright 2026 SIL AEB Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sil_carla_bridge/scenario_publisher.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sil_carla_bridge::ScenarioPublisher>());
  rclcpp::shutdown();
  return 0;
}
