#!/bin/bash

# Ensure the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (use sudo)"
  exit
fi

echo "Starting setup for TurtleBot3 with ROS 2 Humble on a physical robot..."

# Step 1: Update and upgrade the system
echo "Updating system..."
sudo apt update && sudo apt upgrade -y

# Step 2: Install ROS 2 Humble
echo "Installing ROS 2 Humble..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list
sudo apt update
sudo apt install -y ros-humble-ros-base

# Source ROS 2 Humble setup script
echo "Sourcing ROS 2 Humble setup script in ~/.bashrc..."
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source /opt/ros/humble/setup.bash

# Step 3: Install TurtleBot3 packages
echo "Installing TurtleBot3 packages..."
sudo apt install -y ros-humble-turtlebot3* 

# Set the TurtleBot3 model to Burger (you can change to waffle or waffle_pi as needed)
echo "Setting TurtleBot3 model environment variable..."
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc

# Step 4: Create a ROS 2 workspace for TurtleBot3 applications
echo "Setting up workspace for TurtleBot3 applications..."
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Step 5: Clone TurtleBot3 application repositories (for wall-following and other tasks)
echo "Cloning TurtleBot3 application repositories..."
git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_applications.git
git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Step 6: Install dependencies and build the workspace
echo "Installing dependencies and building workspace..."
cd ~/turtlebot3_ws
sudo apt install -y python3-colcon-common-extensions
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# Source the workspace
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Step 7: Final instructions
echo -e "\nSetup complete! To run the wall-following example, use the command:"
echo "source ~/turtlebot3_ws/install/setup.bash"
echo "ros2 launch turtlebot3_applications turtlebot3_wall_follower.launch.py use_sim_time:=false"
