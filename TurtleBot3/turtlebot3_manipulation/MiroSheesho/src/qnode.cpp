#include <ros/ros.h>
#include <string>
#include <vector>
#include "../include/MiroSheesho/qnode.hpp"
#include <ros/network.h>
#include <std_msgs/String.h>
#include <sstream>

namespace MiroSheesho {

QNode::QNode(int argc, char** argv) 
    : init_argc(argc),
      init_argv(argv),
      move_group_arm_(nullptr),
      move_group_gripper_(nullptr)
{}

QNode::~QNode() {
  delete move_group_arm_;
  delete move_group_gripper_;
  if (ros::isStarted()) {
    ros::shutdown();
  }
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "pick_and_place_node");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(1); 
  spinner.start();

  // Initialize MoveGroup interfaces for the arm and gripper
  move_group_arm_ = new moveit::planning_interface::MoveGroupInterface("arm");
  move_group_gripper_ = new moveit::planning_interface::MoveGroupInterface("gripper");

  return true;
}

void QNode::run() {
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    updateRobotState();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void QNode::updateRobotState() {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get current joint values for arm and gripper
  std::vector<double> joint_values_arm = move_group_arm_->getCurrentJointValues();
  std::vector<double> joint_values_gripper = move_group_gripper_->getCurrentJointValues();
  
  // Update joint angles
  present_joint_angle_.clear();
  present_joint_angle_.insert(present_joint_angle_.end(), joint_values_arm.begin(), joint_values_arm.end());
  present_joint_angle_.push_back(joint_values_gripper[0]);

  // Update kinematic position of end effector
  geometry_msgs::Pose current_pose = move_group_arm_->getCurrentPose().pose;
  present_kinematics_position_ = {current_pose.position.x, current_pose.position.y, current_pose.position.z};

  spinner.stop();
}

std::vector<double> QNode::getPresentJointAngle() {
  return present_joint_angle_;
}

std::vector<double> QNode::getPresentKinematicsPosition() {
  return present_kinematics_position_;
}

// bool QNode::setJointSpacePath(const std::vector<double>& joint_angles, double path_time) {
//   move_group_arm_->setJointValueTarget(joint_angles);

//   moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//   bool success = (move_group_arm_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   if (!success) return false;

//   move_group_arm_->move();
//   return true;
// }


bool QNode::setJointSpacePath(const std::vector<double>& joint_angle, double path_time)  // Change to reference
{
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  // Next get the current set of joint values for the group.
  const robot_state::JointModelGroup* joint_model_group =
    move_group_arm_->getCurrentState()->getJointModelGroup("arm");
      
  moveit::core::RobotStatePtr current_state = move_group_arm_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = joint_angle.at(0);  // radians
  joint_group_positions[1] = joint_angle.at(1);  // radians
  joint_group_positions[2] = joint_angle.at(2);  // radians
  joint_group_positions[3] = joint_angle.at(3);  // radians
  move_group_arm_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_arm_->move();

  spinner.stop();
  return true;
}

bool QNode::setTaskSpacePath(const std::vector<double>& kinematics_pose, double path_time) {
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Set position target in Cartesian space
  move_group_arm_->setPositionTarget(kinematics_pose[0], kinematics_pose[1], kinematics_pose[2]);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_arm_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) return false;

  move_group_arm_->move();
  return true;
}

bool QNode::setToolControl(const std::vector<double>& joint_angle) {
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  // Next get the current set of joint values for the group.
  const robot_state::JointModelGroup* joint_model_group =
    move_group_gripper_->getCurrentState()->getJointModelGroup("gripper");
      
  moveit::core::RobotStatePtr current_state = move_group_gripper_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = joint_angle.at(0);  // radians
  move_group_gripper_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_gripper_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_gripper_->move();

  spinner.stop();
  return true;  
}


}  // namespace MiroSheesho
