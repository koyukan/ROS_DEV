// Copyright 2025 Your Name
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "hello_world_robot/talker_node.hpp"
#include "hello_world_robot/listener_node.hpp"

#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class TestHelloWorldIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestHelloWorldIntegration, test_talker_listener_integration)
{
  // Create the nodes
  auto talker = std::make_shared<hello_world_robot::TalkerNode>("test_talker");
  auto listener = std::make_shared<hello_world_robot::ListenerNode>("test_listener");
  
  // Need an executor to process callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(talker);
  executor.add_node(listener);
  
  // Wait for discovery
  std::this_thread::sleep_for(10000ms);
  
  // Initial check - no messages yet
  EXPECT_EQ(listener->get_message_count(), 0);
  
  // Publish messages
  for (int i = 0; i < 5; i++) {
    talker->publish_message();
    
    // Process some events
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && 
           std::chrono::steady_clock::now() - start < 100ms) {
      executor.spin_some();
    }
  }
  
  // Final check - some messages should have been received
  EXPECT_GT(listener->get_message_count(), 0);
  EXPECT_TRUE(listener->get_last_message().find("Hello World") != std::string::npos);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
