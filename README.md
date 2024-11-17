
# TurtleBot3 Documentation

This documentation provides a complete guide for setting up, simulating, and operating TurtleBot3. It includes the installation of necessary software, simulation and hardware configuration, and troubleshooting common issues. This guide consolidates information from official TurtleBot3 resources and additional insights to ensure a thorough understanding and efficient execution of projects.

---

## Overview

### Simulation Setup
- Teleoperation
- SLAM and navigation
- Manipulation in Gazebo simulation
- Visualization through RViz simulator

### Hardware Setup
- SLAM implementation in a pre-mapped environment
- Navigation hardware configuration and tuning
- Manipulator setup and troubleshooting

### Key Tools and Dependencies
- **Gazebo:** For simulation
- **RViz:** For visualization
- **APF package:** For navigation
- **OpenCR:** For manipulator operations

---

## Turtlebot3 Packages Installation and Setup

Before installing the Turtlebot3 related packages, make sure that ROS Noetic is already installed.

Open the terminal with Ctrl+Alt+T and enter below commands one at a time

### 1. Turtlebot3 ROS Packages Installation

```bash
sudo apt install ros-noetic-dynamixel-sdk
```

```bash
sudo apt install ros-noetic-turtlebot3-msgs
```

```bash
sudo apt install ros-noetic-turtlebot3
```

### 2. Turtlebot3 Setup

#### 1. Clone this Git Repository
```bash
cd ~/catkin_ws
git clone -b Turtlebot_3 https://github.com/Sheero-git/IAS_Nov-24_WS.git
```

#### 2. Build the Ros Environment using catkin make
````bash
catkin_make
````

#### 3. Change the bashrc file to include the correct Turtlebot model & Lidar model
##### 3.1 Open the bashrc file
````bash
cd
nano ~/.bashrc
````
##### 3.2 Add these export lines to the bottom of the file
````bash
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-02
````
Save the bashrc file using (`ctrl` + `S`) then exit using (`ctrl` + `X`)

##### 3.3 Source the bashrc file
````bash
source ~/.bashrc
````
   

## Launching the TurtleBot3 Gazebo Simulation

### 1. Load TurtleBot3 with OpenMANIPULATOR-X into Gazebo world
```bash
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```

### 2. Load packages for navigation, manipulator and integration between them
```bash
roslaunch sim_turtle_bot my_launch_file.launch
```

### 3. Click start button down in gazebo (Play button)

### 4. (Optional) To control the manipulator through the default gazebo gui 
```bash
roslaunch turtlebot3_manipulation_gui turtlebot3_manipulation_gui.launch
```

## Moving actual TurtleBot (Hardware):

### 1. Setup the master on the Turtlebot (Done Only Once)

#### 1.1 Check your [Remote PC]'s ip adress using ifconfig
````bash
ifconfig
````
##### 1.2 Get your ip address beside the inet addr in the wlp2s0 section

#### 1.3 [Remote PC] SSH the Turtlebot Pi using it's IP Address (currently set to 192.168.0.123)
```bash
ssh ubuntu@192.168.0.123
```

#### 1.4 Enter the password
```bash
turtlebot
```
#### 1.5 Open the bashrc file on the turtlebot
````bash
cd
nano ~/.bashrc
````

#### 1.6 Change the ROS_MASTER_URI to be your [Remote PC]'s ip address
````bash
export ROS_MASTER_URI=http://${YOUR_IP_ADDRESS}:11311
````
Save the bashrc file using (`ctrl` + `S`) then exit using (`ctrl` + `X`)

##### 1.7 Source the bashrc file
````bash
source ~/.bashrc
````

### 2. [Remote PC]  Run Ros
```bash
roscore
```

### 3. [Remote PC] SSH the Turtlebot Pi unsing it's IP Address (currently set to 192.168.0.123)
```bash
ssh ubuntu@192.168.0.123
```
#### 3.1 Enter the password
```bash
turtlebot
```

### 4. [TurtleBot3 SBC] Run Bringup node for TurtleBot3
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

### 5. [Remote PC]  Run Bringup node for OpenMANIPULATOR on TurtleBot3
```bash
roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```

### 6. Load packages for navigation, manipulator and integration between them
```bash
roslaunch sim_turtle_bot my_launch_file.launch
```

---

## Further Explaination and Parameters Editing

### 1. SLAM
- SLAM was implemented in a pre-mapped environment, ensuring robust operation without the need for real-time mapping.

### 2. Navigation
- Hardware navigation parameters were tuned for optimal performance (Found in Node: src/turtlebot_architecture/src/APF.py).
- Obstacle avoidance was verified and refined through extensive testing.
- Goal Setup (src/scenario/src/Scenario.py)

### 3. Manipulation
- Manipulation setup utilized OpenCR for operational guidance.
- Pickup and Place joint angles Setup (src/turtlebot3_manipulation/MiroSheesho/src/test.cpp)


#### Key Challenges and Solutions:
1. **Timing Discrepancies:**
   - System clock mismatches between the TurtleBot's Raspberry Pi and the remote PC caused unrealistic time differences.
   - **Solution:** Used `timedatectl` to align time zones and resolved time mismatch issues.

2. **Command Abortion Errors:**
   - Fixed by adjusting planning and execution time bounds to initialize proper states.

---

## [Optional] Launching TurtleBot nodes separately 

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
