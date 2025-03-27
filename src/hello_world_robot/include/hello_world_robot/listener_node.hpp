// Copyright 2025 Your Name
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef HELLO_WORLD_ROBOT__LISTENER_NODE_HPP_
#define HELLO_WORLD_ROBOT__LISTENER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string>

namespace hello_world_robot
{

class ListenerNode : public rclcpp::Node
{
public:
  explicit ListenerNode(const std::string & node_name = "listener_node")
  : Node(node_name), last_message_(""), message_count_(0)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "hello_topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        last_message_ = msg->data;
        message_count_++;
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }

  // Public methods to check the state (useful for testing)
  std::string get_last_message() const
  {
    return last_message_;
  }

  int get_message_count() const
  {
    return message_count_;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string last_message_;
  int message_count_;
};

}  // namespace hello_world_robot

#endif  // HELLO_WORLD_ROBOT__LISTENER_NODE_HPP_
