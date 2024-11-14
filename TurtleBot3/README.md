# TurtleBot3

## Installation

Follow the steps below to install ROS Noetic and the necessary packages for your project.

### 1. Install ROS Noetic

Open a terminal (Ctrl+Alt+T) and enter the following commands one by one:
```
# Update the package list
sudo apt update

# Upgrade installed packages
sudo apt upgrade

# Download the ROS Noetic installation script
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh

# Set executable permissions for the script
chmod 755 ./install_ros_noetic.sh

# Run the installation script
bash ./install_ros_noetic.sh

### 2. Install Dependent ROS Packages
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
### 3. Install Dependent ROS Packages

# Install Dynamixel SDK
sudo apt install ros-noetic-dynamixel-sdk

# Install TurtleBot3 message packages
sudo apt install ros-noetic-turtlebot3-msgs

# Install TurtleBot3 packages
sudo apt install ros-noetic-turtlebot3

### Launching the TurtleBot3 Gazebo Simulation


## Hardware 
