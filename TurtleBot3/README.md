
# TurtleBot3 Documentation

This documentation provides a complete guide for setting up, simulating, and operating TurtleBot3. It includes the installation of necessary software, simulation and hardware configuration, and troubleshooting common issues. This guide consolidates information from official TurtleBot3 resources and additional insights to ensure a thorough understanding and efficient execution of projects.

---

## Overview

### Simulation Setup
- Teleoperation
- SLAM and navigation
- Manipulation in Gazebo simulation

### Hardware Setup
- SLAM implementation in a pre-mapped environment
- Navigation hardware configuration and tuning
- Manipulator setup and troubleshooting

### Key Tools and Dependencies
- **Gazebo:** For simulation
- **RViz:** For visualization
- **SLAM packages:** For mapping and navigation
- **OpenCR:** For manipulator operations

---

## ROS Installation

Follow the steps below to install ROS Noetic and the necessary packages for your project.

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

## Turtlebot3 Packages Installation

```bash
sudo apt install ros-noetic-dynamixel-sdk
```

```bash
sudo apt install ros-noetic-turtlebot3-msgs
```

```bash
sudo apt install ros-noetic-turtlebot3
```

```bash
git clone -b Turtlebot_3 https://github.com/Sheero-git/IAS_Nov-24_WS.git
```

## Launching the TurtleBot3 Gazebo Simulation

### 1. Load TurtleBot3 with OpenMANIPULATOR-X into Gazebo world
```bash
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```

### 2. Load packages for navigation, manipulator and integration between them
```bash
roslaunch sim_turtle_bot my_launch_file.launch
```

### 3. Click start button down in gazebo


## Moving actual TurtleBot

### 1. [Remote PC] SSH the Turtlebot Pi unsing it's IP Address (currently set to 192.168.0.123
```bash
ssh ubuntu@192.168.0.123
```
#### 1.1 Enter the password
```bash
turtlebot
```

### 2. [TurtleBot3 SBC] Run Bringup node for TurtleBot3
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### 3. [Remote PC]  Run Bringup node for OpenMANIPULATOR on TurtleBot3
```bash
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```

### 4. Load packages for navigation, manipulator and integration between them
```bash
roslaunch sim_turtle_bot my_launch_file.launch
```

---

## Hardware Setup

### 1. SLAM on Hardware
- SLAM was implemented in a pre-mapped environment, ensuring robust operation without the need for real-time mapping.

### 2. Navigation on Hardware
- Hardware navigation parameters were tuned for optimal performance.
- Obstacle avoidance was verified and refined through extensive testing.

### 3. Manipulation on Hardware
- Manipulation setup utilized OpenCR for operational guidance.

#### Key Challenges and Solutions:
1. **Timing Discrepancies:**
   - System clock mismatches between the TurtleBot's Raspberry Pi and the remote PC caused unrealistic time differences.
   - **Solution:** Used `timedatectl` to align time zones and resolved time mismatch issues.

2. **Command Abortion Errors:**
   - Fixed by adjusting planning and execution time bounds to initialize proper states.

---

## Launching Necessary Nodes for TurtleBot3

### Navigation:
```bash
roslaunch turtlebot_architecture Turtlecontrol1.launch
```

### Manipulator

```bash
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
rosrun MiroSheesho MiroSheesho
```

### Scenario

```bash
rosrun scenario Scenario.py
```

## Troubleshooting

### 1. RViz vs. Gazebo Consistency
- RViz outcomes cannot be assumed based solely on Gazebo simulation results.
- Ensure separate validations for consistency.

### 2. Time Synchronization
- Manual synchronization was required using `timedatectl`.
- Future automation with reliable tools is recommended.

### 3. Initial State Management
- Proper configuration of initial state parameters avoids command abortion errors.

---

## Lessons Learned

- **Time Synchronization:** Accurate system clocks are critical for seamless hardware operations.
- **Hardware Tuning:** Navigation and manipulation success depend heavily on precise parameter adjustments.
- **Separate Validations:** RViz and Gazebo should be independently validated for reliability.

---

## Future Improvements

1. Automate time synchronization to minimize manual intervention.
2. Enhance RViz-Gazebo integration for improved manipulation outcomes.
3. Expand testing scenarios for robustness in real-world applications.
