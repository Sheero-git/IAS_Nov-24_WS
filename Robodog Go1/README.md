# Robodog Go1 Unitree Project

This project is focused on integrating the **Robodog Go1** quadruped robot by Unitree with **ROS** for high-level control, navigation, and path planning. It includes a **Finite State Machine (FSM)** to manage robot modes, navigation control, path planning, and testing in both simulation and on real hardware.

## Table of Contents

- [Project Overview](#project-overview)
- [Navigation](#navigation)
  - [High-Level Control](#high-level-control)
  - [Low-Level Control](#low-level-control)
- [Path Planning](#path-planning)
- [Hardware Integration](#hardware-integration)
- [Simulation Setup](#simulation-setup)
- [Contributing](#contributing)

## Project Overview

The **Robodog Go1** operates through an FSM with multiple modes and integrates with ROS for autonomous navigation and control. The project includes simulation (Gazebo) and real-hardware testing phases.

## Navigation

### High-Level Control

High-level navigation is managed by sending commands over ROS topics. Key topics include:

- **/cmd_vel**: Controls movement via `Twist` messages.
- **/unitree_gazebo_servo**: Manages joint control for movement.

Commands are sent from a main computer to the **Robodog’s Raspberry Pi** over WiFi, utilizing `HighCmd` and `HighState` messages for communication.

### Low-Level Control

Low-level control leverages Unitree’s SDK to manage joint positions and gait, simplifying control of the 12 individual joints.

# Path Planning for Robodog Go1

The path planning process uses for Hardware: `PathPlanning_SH` and `move_robot_SH` nodes. For Simulation: `PathPlanning_states` and `move_robot_states`to navigate the robot towards a target while avoiding obstacles.

## Flow Breakdown

1. **Input Data**:
   - **/move_robot/goal**: Target goal position.
   - **/gazebo/model_states**: Robot's current position. (In hardware we use /odom)
   - **/obstacles**: Obstacle data. (Simulation only)

2. **Path Calculation**:
   - `path_planner` evaluates the goal, current position, and obstacles to create a feasible path and divides this path to multiple points and publishes each point on /move_robot/target and waits till the robodog reaches this point then publishes the next point.

3. **Waypoint Execution**:
   - Waypoints are sent to `move_robot`, which converts them to velocity commands on **/cmd_vel**.

4. **Goal Monitoring**:
   - If the robot reaches the final waypoint, the robodog stops.

## ROS Topics

### `move_robot_states` Node
- **Publisher**: `/cmd_vel` - Velocity commands.
- **Subscribers**: `/move_robot/target` (waypoints), `/move_robot/relative_point`, `/gazebo/model_states`.

### `path_planner` Node
- **Publishers**: `/move_robot/target` (waypoints).
- **Subscribers**: `/move_robot/goal`, `/move_robot/relative_goal`, Simulation:(`/gazebo/model_states`, `/obstacles`.), Hardware: high_state_position
#### Note:
- We used `/gazebo/model_states` in simulation instead of /odom because /odom gives inaccurate position in Gazebo

## Hardware Integration

The **Robodog Go1** hardware integrates with an **Open Manipulator X** through a **u2d2 board** for additional control capabilities. WiFi connectivity allows remote control. Network and ROS communication were addressed by setting up the **Unitree SDK**.

Important hardware ROS topics:

- **HighCmd**: Commands for movement, mode, and speed.
- **HighState**: State data such as position and IMU readings.

## Simulation Setup

In Gazebo, a custom world is used for navigation and obstacle avoidance tests, with modes available to simulate the robot's stance and mobility. 

To run the simulation, clone the [GitHub repository](https://github.com/macc-n/ros_unitree.git), build the workspace, and launch the simulation with:
```bash
roslaunch unitree_gazebo robot_simulation.launch rname:=go1 wname:=earth
```
Clone our repository [GitHub repository](https://github.com/Sheero-git/IAS_Nov-24_WS.git)
Run the controller:
```bash
rosrun unitree_guide junior_ctrl
```
You should activate MoveBase mode by pressing keys '2' and '5'.

For path planning:
Open separate terminals for:
```bash
rosrun ros_unitree_moving move_robot_states.py
rosrun ros_unitree_moving PathPlanning_states.py
rosrun ros_unitree_moving obstacles.py
```
Publish a goal (example):
```bash
rostopic pub /move_robot/goal geometry_msgs/Point "x: 3.0 y: 4.0 z: 0.0"
```
