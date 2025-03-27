
# Useful Commands
- Test if the ROS2 environment is working
```bash

sudo apt install ros-$ROS_DISTRO-rviz2 -y
source /opt/ros/$ROS_DISTRO/setup.bash
rviz2

```

## Build and Run the project
```bash
colcon build --packages-select hello_world_robot
```

### Run the test 
```bash
colcon test --packages-select hello_world_robot --event-handlers console_direct+
```


### Run the talker 
```bash
ros2 run hello_world_robot talker_node
```
### Run the listener
```bash
ros2 run hello_world_robot listener_node
```



# Resources
- Created a dev container for the project
https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html

