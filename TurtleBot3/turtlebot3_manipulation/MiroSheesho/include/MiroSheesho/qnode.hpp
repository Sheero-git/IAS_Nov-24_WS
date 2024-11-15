/*******************************************************************************
* Simplified qnode.hpp without Qt dependencies
*******************************************************************************/

#ifndef MIROSHEESHO_QNODE_HPP_
#define MIROSHEESHO_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.h>

namespace MiroSheesho {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode {
public:
  QNode(int argc, char** argv);
  virtual ~QNode();

  bool init();
  void run();

  void updateRobotState();
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPosition();

  bool setJointSpacePath(const std::vector<double>& joint_angle, double path_time);
  bool setTaskSpacePath(const std::vector<double>& kinematics_pose, double path_time);
  bool setToolControl(const std::vector<double>& joint_angle);

private:
  int init_argc;
  char** init_argv;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematics_position_;

  moveit::planning_interface::MoveGroupInterface* move_group_arm_;
  moveit::planning_interface::MoveGroupInterface* move_group_gripper_;
};

}  // namespace MiroSheesho

#endif /* MIROSHEESHO_QNODE_HPP_ */
