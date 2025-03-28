# ROS 2 Hello World Robot: Publisher/Subscriber with CI/CD

A demonstrative ROS 2 project featuring a publisher-subscriber communication system with complete development, testing, and deployment pipeline.

## Project Overview

This project demonstrates a complete ROS 2 development workflow:

1. **Development Environment**: Containerized with Docker and VS Code
2. **ROS 2 Nodes**: Simple publisher and subscriber communicating via topics
3. **Testing**: Integration tests that verify node communication
4. **CI/CD Pipeline**: GitHub Actions for automated building, testing, and deployment to AWS ECR

## Development Environment

This project uses a containerized development environment via Docker and VS Code's Remote-Containers extension. This provides:

- Consistent development environment across different machines
- Isolated ROS 2 installation that doesn't interfere with the host system
- Easy onboarding for new developers

To use the development container:

1. Install [Docker](https://docs.docker.com/get-docker/) and [VS Code](https://code.visualstudio.com/)
2. Install the [Remote-Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
3. Clone this repository
4. Open the repository in VS Code
5. When prompted, select "Reopen in Container" or use the command palette (F1) and select "Remote-Containers: Reopen in Container"

## Project Structure

```
.
├── .devcontainer/           # Dev container configuration
├── .github/workflows/       # GitHub Actions CI/CD configuration
├── include/                 # C++ header files
│   └── hello_world_robot/
│       ├── listener_node.hpp
│       └── talker_node.hpp
├── src/                     # Source code
│   ├── listener_node.cpp    # Subscriber implementation
│   └── talker_node.cpp      # Publisher implementation
├── test/                    # Test files
│   └── test_hello_world.cpp # Integration test
├── CMakeLists.txt           # Build configuration
├── Dockerfile               # Container image for deployment
├── package.xml              # Package metadata
└── README.md                # This file
```

## Node Functionality

This project implements two ROS 2 nodes:

1. **Talker Node**: Publishes "Hello World" messages with an increasing counter
2. **Listener Node**: Subscribes to these messages and logs them when received

The nodes communicate over the `hello_topic` topic using standard `std_msgs/String` messages.

## Building and Running

### Build the Package

```bash
# Inside the dev container
cd /home/ws
colcon build --packages-select hello_world_robot
source install/setup.bash
```

### Run the Talker Node

```bash
ros2 run hello_world_robot talker_node
```

You should see output like:
```
[INFO] [1712345678.123456789] [talker_node]: Publishing: 'Hello World: 0'
[INFO] [1712345679.123456789] [talker_node]: Publishing: 'Hello World: 1'
...
```

### Run the Listener Node

In another terminal:
```bash
# Source the setup file first
source /home/ws/install/setup.bash
ros2 run hello_world_robot listener_node
```

You should see output like:
```
[INFO] [1712345680.123456789] [listener_node]: I heard: 'Hello World: 2'
[INFO] [1712345681.123456789] [listener_node]: I heard: 'Hello World: 3'
...
```

## Testing

The project includes integration tests that verify the talker and listener nodes can communicate correctly.

### Run the Tests

```bash
# Inside the dev container
cd /home/ws
colcon test --packages-select hello_world_robot --event-handlers console_direct+
```

The test creates instances of both nodes, checks that messages are published correctly, and verifies that the listener receives them.

## CI/CD Pipeline

This project uses GitHub Actions for continuous integration and deployment:

1. **Build**: The ROS 2 package is built in a container
2. **Test**: Integration tests are run to verify functionality
3. **Deploy**: On successful builds of the main branch, a Docker image is built and pushed to Amazon ECR

The pipeline is defined in `.github/workflows/ros-ci.yml`.

### AWS Deployment

The Docker image is pushed to Amazon ECR at:
```
404883402772.dkr.ecr.us-east-1.amazonaws.com/ros_test
```

The Docker image contains both nodes and is configured to run them both when started.

## Testing the Deployment

To pull and run the deployed container:

```bash
# Authenticate with AWS ECR
aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin 404883402772.dkr.ecr.us-east-1.amazonaws.com

# Pull the latest image
docker pull 404883402772.dkr.ecr.us-east-1.amazonaws.com/ros_test:latest

# Run the container
docker run -it 404883402772.dkr.ecr.us-east-1.amazonaws.com/ros_test:latest
```

The container will automatically start both the talker and listener nodes and you'll see their communication in the output.

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/rolling/)
- [ROS 2 with VS Code and Docker Container Setup Guide](https://docs.ros.org/en/iron/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
- [ROS 2 Publisher/Subscriber Tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [AWS ECR Documentation](https://docs.aws.amazon.com/ecr/)

## Troubleshooting

### ROS 2 Environment Issues

If you encounter issues with ROS 2 commands not being found, make sure you've sourced the setup files:

```bash
source /opt/ros/rolling/setup.bash
source /home/ws/install/setup.bash
```

### Testing the ROS 2 Environment

To verify your ROS 2 environment is working correctly:

```bash
sudo apt install ros-$ROS_DISTRO-rviz2 -y
source /opt/ros/$ROS_DISTRO/setup.bash
rviz2
```

If RViz2 launches, your ROS 2 environment is correctly set up.
