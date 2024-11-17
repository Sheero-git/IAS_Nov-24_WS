# IAS_Nov'24_WS

This repository documents our work with three different robots—TurtleBot, Robodog Go1, and Franka Emika Panda—during the November 2024 workshop at the University of Stuttgart. We are students from the German University in Cairo, and this project showcases our work and collaboration on these robotic platforms.

To streamline development and documentation, the repository is divided into three separate branches, each dedicated to a specific robot. Each branch contains code, launch files, and detailed documentation for the respective robot's setup, nodes, and usage.
Branches

## [TurtleBot 3](https://github.com/Sheero-git/IAS_Nov-24_WS/tree/Turtlebot_3)

To clone this branch:  
```bash
git clone -b Turtlebot_3 https://github.com/Sheero-git/IAS_Nov-24_WS.git
```

## [Franka Emika Panda](https://github.com/Sheero-git/IAS_Nov-24_WS/tree/Franka-Emica-Panda)

To clone this branch:  
```bash
git clone -b Franka-Emica-Panda https://github.com/Sheero-git/IAS_Nov-24_WS.git
```

## [Robodog Go1](https://github.com/Sheero-git/IAS_Nov-24_WS/tree/Robodog-Go1)

To clone this branch:   
```bash
git clone -b Robodog-Go1 https://github.com/Sheero-git/IAS_Nov-24_WS.git
```

## Documentation

Each branch contains its own README.md file with specific details on the nodes, launch files, and robot configuration. Please refer to these files in each branch for a comprehensive understanding of the setup and usage instructions.
...

## Start Up: ROS Installation

Follow the steps below to install ROS Noetic and the necessary packages if not installed prior to this project.

### 1. Install ROS Noetic

Open a terminal (Ctrl+Alt+T) and enter the following commands one by one:

Update the package list
```bash
sudo apt update
```

Upgrade installed packages
```bash
sudo apt upgrade
```

Download the ROS Noetic installation script
```bash
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
```

Set executable permissions for the script
```bash
chmod 755 ./install_ros_noetic.sh
```

Run the installation script
```bash
bash ./install_ros_noetic.sh
```

### 2. Install Dependent ROS Packages

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
### 3. ROS Sourcing 
In order to make ROS Source as a default, Open your bashrc 

```bash
nano ~/.bashrc
```
Add the following commands at the end of the bashrc terminal
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```
save your edits in the bashrc (Ctrl+S), then exit this terminal (Ctrl+x)
