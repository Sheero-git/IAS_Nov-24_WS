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

### 1. Install ROS Noetic
Follow these steps to install ROS Noetic on your system:

```bash
# Update and upgrade package list
sudo apt update
sudo apt upgrade

# Download the ROS Noetic installation script
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh

# Set executable permissions and run the script
chmod 755 ./install_ros_noetic.sh
bash ./install_ros_noetic.sh

2. Install Dependent ROS Packages
Run the following commands to install the necessary ROS packages:

# General dependencies
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

# TurtleBot-specific dependencies
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3

## Simulation Setup

### 1. Launching the TurtleBot3 Gazebo Simulation
Use the following command to launch the Gazebo simulation:
```bash
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch

## Simulation Setup

### 2. Teleoperation
- Teleoperation was successfully configured in the simulation environment for remote control.

### 3. SLAM and Navigation
- SLAM and navigation functionality were verified in the simulation environment with full obstacle avoidance and map generation.

### 4. Manipulation in Gazebo
- Manipulation tasks were tested in Gazebo with the following considerations:
  - **Issue:** RViz sometimes fails or reports execution errors for long-distance manipulations.
  - **Solution:** Additional checks in RViz state were performed to ensure consistency.

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

### Manipulator
```bash
roslaunch turtlebot3_manipulation_moveit_config move_group.launch
rosrun MiroSheesho MiroSheesho

## Scenario

```bash
rosrun scenario Scenario.py

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
