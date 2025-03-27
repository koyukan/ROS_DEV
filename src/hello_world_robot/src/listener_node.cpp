// Copyright 2025 Your Name
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "hello_world_robot/listener_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hello_world_robot::ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
