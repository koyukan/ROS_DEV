cmake_minimum_required(VERSION 3.8)
project(hello_world_robot)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

# Add talker_node executable
add_executable(talker_node src/talker_node.cpp)
ament_target_dependencies(talker_node rclcpp std_msgs)

# Add listener_node executable
add_executable(listener_node src/listener_node.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)

# Install executables
install(TARGETS
  talker_node
  listener_node
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  
  # Include linters
  #ament_lint_auto_find_test_dependencies()
  #find_package(ament_lint_auto REQUIRED)
  
  # Add the integration test
  ament_add_gtest(test_hello_world test/test_hello_world.cpp)
  target_include_directories(test_hello_world PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_hello_world
    rclcpp
    std_msgs)
endif()

ament_package()