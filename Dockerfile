FROM ros:rolling

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-colcon-package-information \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros_ws

# Copy package files
COPY . /ros_ws/src/hello_world_robot/

# Build package
RUN . /opt/ros/rolling/setup.sh && \
    colcon build --packages-select hello_world_robot

# Create a launch script
RUN echo '#!/bin/bash \n\
source "/opt/ros/rolling/setup.bash" \n\
source "/ros_ws/install/setup.bash" \n\
echo "Starting ROS nodes..." \n\
ros2 run hello_world_robot talker_node &  \n\
ros2 run hello_world_robot listener_node \n\
' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]