#!/bin/bash

# Ensure the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (use sudo)"
  exit
fi

echo "Starting setup for TurtleBot3 wall-avoidance program on a physical robot..."

# Step 1: Update the system
echo "Updating system..."
sudo apt update && sudo apt upgrade -y

# Step 2: Install ROS 2 packages if they are not installed
UBUNTU_VERSION=$(lsb_release -sc)
if [ "$UBUNTU_VERSION" == "focal" ]; then
  ROS_DISTRO="foxy"
elif [ "$UBUNTU_VERSION" == "jammy" ]; then
  ROS_DISTRO="humble"
else
  echo "Unsupported Ubuntu version. This script only supports 20.04 (Focal) and 22.04 (Jammy)."
  exit
fi

if ! command -v ros2 &> /dev/null; then
  echo "Installing ROS 2 $ROS_DISTRO..."
  sudo apt install -y ros-$ROS_DISTRO-robot
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
  source /opt/ros/$ROS_DISTRO/setup.bash
fi

# Step 3: Install TurtleBot3 packages if not already installed
if ! dpkg -l | grep -q ros-$ROS_DISTRO-turtlebot3; then
  echo "Installing TurtleBot3 packages..."
  sudo apt install -y ros-$ROS_DISTRO-turtlebot3* 
  echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
  source ~/.bashrc
fi

# Step 4: Create a ROS 2 workspace for the wall-avoidance program
echo "Setting up workspace for wall-avoidance..."
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Step 5: Clone a wall-avoidance ROS 2 package
echo "Cloning TurtleBot3 wall-avoidance package..."
git clone https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
cd turtlebot3_applications
git checkout ros2
cd ..

# Step 6: Install dependencies and build the workspace
echo "Installing dependencies and building workspace..."
sudo apt install -y python3-colcon-common-extensions
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Source the workspace
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Step 7: Launch the wall-avoidance node for a physical TurtleBot3
echo "Launching TurtleBot3 wall-avoidance node..."
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_applications turtlebot3_wall_follower.launch.py use_sim_time:=false &
