// Copyright 2025 Your Name
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef HELLO_WORLD_ROBOT__TALKER_NODE_HPP_
#define HELLO_WORLD_ROBOT__TALKER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <string>

namespace hello_world_robot
{

class TalkerNode : public rclcpp::Node
{
public:
  explicit TalkerNode(const std::string & node_name = "talker_node")
  : Node(node_name), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("hello_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000), 
      std::bind(&TalkerNode::timer_callback, this));
  }

  // Public method to publish a single message (useful for testing)
  void publish_message()
  {
    timer_callback();
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

}  // namespace hello_world_robot

#endif  // HELLO_WORLD_ROBOT__TALKER_NODE_HPP_
