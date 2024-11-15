#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import moveit_commander
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

def forward_kin(l12,A,l3,l4,q2,q3,q4):
    
    w =  l12*np.sin(q2+A) + l3*np.cos(q2+q3) + l4*np.cos(q2+q3+q4) 
    z =  l12*np.cos(q2+A) - l3*np.sin(q2+q3) - l4*np.sin(q2+q3+q4) 
    return np.array([[w],[z]])

def wrap_2_pi(q):
    for i in range(3):
        while(q[i,0] > 1.57):
            q[i,0]-=3.14
        while(q[i,0]<-1.57):
            q[i,0]+=3.14    


def jacobian_Matrix(l12,A,l3,l4,q2,q3,q4):
    
    dwdq2 = - l12*np.cos(q2+A) - l3*np.sin(q2+q3) - l4*np.sin(q2+q3+q4)
    dwdq3 = - l3*np.sin(q2+q3) - l4*np.sin(q2+q3+q4)
    dwdq4 = - l4*np.sin(q2+q3+q4)

    dzdq2 = - l12*np.sin(q2+A) - l3*np.cos(q2+q3) - l4*np.cos(q2+q3+q4)
    dzdq3 = - l3*np.cos(q2+q3) - l4*np.cos(q2+q3+q4)
    dzdq4 = - l4*np.cos(q2+q3+q4)

    dadq2 = 1
    dadq3 = 1
    dadq4 = 1
    
    # return np.array([[dwdq2,dwdq3,dwdq4],[dzdq2,dzdq3,dzdq4],[dadq2,dadq3,dadq4]])
    return np.array([[dwdq2,dwdq3,dwdq4],[dzdq2,dzdq3,dzdq4]])


def newton_raphson_inverse(x,y,z,alpha):
    l1 = 0.128
    l2 = 0.024
    l3 = 0.124
    l4 = 0.126

    l12 = np.sqrt(l1**2 + l2**2)
    A = np.arctan2(l2,l1)

    q1 = np.arctan2(y,x)
    w_goal = np.sqrt(x**2 + y**2)
    z_goal = z   

    # goal = forward_kin(l12,A,l3,l4,0,0,0)
    # w_goal = goal[0,0]
    # z_goal = goal[1,0]
    # alpha = goal[2,0]
    joint_values = move_group_arm.get_current_joint_values()
    q = np.array([[joint_values[1]],[joint_values[2]],[joint_values[3]]])
    print("Desired goal: ",w_goal," ", z_goal," ", alpha)

    for i in range(1000):
        q2 = q[0,0]
        q3 = q[1,0]
        q4 = q[2,0]
        forward = forward_kin(l12,A,l3,l4,q2,q3,q4)
        # error = forward - np.array([[w_goal],[z_goal],[alpha]])
        error = forward - np.array([[w_goal],[z_goal]])
        jacobian = jacobian_Matrix(l12,A,l3,l4,q2,q3,q4)
        q = q - np.linalg.pinv(jacobian).dot(error)
        wrap_2_pi(q)
        

    return q1 , q[0,0], q[1,0], q[2,0]


def inverse_kinematics_RRR(x, y, z, alpha):
    l1 = 0.128
    l2 = 0.024
    l3 = 0.124
    l4 = 0.126

    l12 = np.sqrt(l1**2 + l2**2)
    A = np.arctan2(l2,l1)

    w = np.sqrt(x**2 + y**2)
    q1 = np.arctan2(y,x)
    z_prime = z + l4*np.cos(alpha)
    w_prime = w - l4*np.sin(alpha)

    # z_prime = z
    # w_prime = w

    cos_beta = ( w_prime**2 + z_prime**2 - l12**2 - l3**2 )/(2*l12*l3)
    print("cos_beta: ",cos_beta)
    beta = np.arccos(cos_beta) 
    q3 = np.deg2rad(90)-(beta-A)

    cos_q2 = (( l12*np.sin(A) + l3*np.cos(q3))*w - (l12*np.cos(A)-l3*np.sin(q3))*z  )/(w_prime**2 + z_prime**2)
    sin_q2 = (( l12*np.cos(A) + l3*np.sin(q3))*w - (l12*np.sin(A)+l3*np.cos(q3))*z  )/(w_prime**2 + z_prime**2)

    q2 = np.arctan2(sin_q2,cos_q2)
#     # Adjust theta3
#     theta3 = phi - theta1 - theta2

    q4 = alpha-q2-q3
    print("Old thetas: ",q2, q3, q4)


    return q1,q2,q3,q4

# def inverse_kinematics_RRR(x, y, z, phi):
#     l0 = 0.128
#     l1 = 0.024
#     l2 = 0.124
#     l3 = 0.126

#     print(" ------------------------------------------------------ ")
#     print(" ------------------------------------------------------ ")
#     print(" ------------------------------------------------------ ")

#     print(x , y , z, phi)

#     w = np.sqrt(x**2 +y**2)
#     theta0 = np.arctan2(y,x)
    
#     l01 = np.sqrt(l0**2 +l1**2)
#     theta1_offset = np.arctan2(l1,l0)

#     # Calculate theta3
#     theta3 = phi

#     # Effective position for 2R problem
#     w_prime = w - l3 * np.cos(phi)
#     z_prime = z - l3 * np.sin(phi)

#     # Calculate theta2
#     cos_theta2 = (w_prime**2 + z_prime**2 - l01**2 - l2**2) / (2 * l01 * l2)
#     sin_theta2 = np.sqrt(1 - (cos_theta2**2))  # Consider both positive and negative for sin_theta2

#     print("cos_theta2: ",cos_theta2, "sin_theta2: ",sin_theta2)

#     theta2 = np.arctan2(sin_theta2, cos_theta2)  # First possible solution
#     # Alternate solution for theta2
#     # theta2 = np.arctan2(-sin_theta2, cos_theta2)

#     # Calculate theta1
#     k1 = l01 + l2 * cos_theta2
#     k2 = l2 * sin_theta2
#     theta1 = np.arctan2(z_prime, w_prime) - np.arctan2(k2, k1)
#     print("z_prime: ",z_prime, "w_prime: ",w_prime, "k1: ",k1,"k2: ",k2 )

#     # Adjust theta3
#     theta3 = phi - theta1 - theta2

#     print("Old thetas: ",theta1, theta2, theta3)

#     # theta1 += np.deg2rad(90)-theta1_offset
#     # theta2 = -(np.deg2rad(90) - theta2)

#     # theta3 = -theta3


#     theta1 = np.deg2rad(90)-theta1
#     theta2 = -(np.deg2rad(90) - theta2)

#     theta3 = -theta3

#     return theta0, theta1, theta2, theta3

# # Example usage
# l1, l2, l3 = 1.0, 1.0, 1.0  # Link lengths
# x, y = 1.5, 1.5  # Desired end-effector position
# phi = np.pi / 4  # Desired end-effector orientation (45 degrees)

# theta1, theta2, theta3 = inverse_kinematics_RRR(l1, l2, l3, x, y, phi)
# print(f"Theta1: {np.degrees(theta1):.2f} degrees")
# print(f"Theta2: {np.degrees(theta2):.2f} degrees")
# print(f"Theta3: {np.degrees(theta3):.2f} degrees")


def TaskSpace_Goal_Subscriber(pose_goal:Pose):
    inverse_Kinematics = True
    print("============ Printing Position ")
    pose = move_group_arm.get_current_pose().pose
    
    print("Pose: ",pose)
    goal_position = pose_goal.position
    goal_orientation = pose_goal.orientation.w

    if(inverse_Kinematics):
        

        q1,q2,q3,q4 = newton_raphson_inverse(goal_position.x,goal_position.y,goal_position.z,goal_orientation)
        
        desired_joints = JointState()

        desired_joints.position=[q1,q2,q3,q4]
        print("Desired_joints: ", desired_joints.position)
        JointSpace_Goal_Subscriber(desired_joints)

    if(not inverse_Kinematics):
        quat_orientation= quaternion_from_euler(0,goal_orientation,0)

        pose_goal.orientation.x = quat_orientation[0]
        pose_goal.orientation.y = quat_orientation[1]
        pose_goal.orientation.z = quat_orientation[2]
        pose_goal.orientation.w = quat_orientation[3]
        # pose_goal = pose
        # pose_goal.position.z+=0.05
        print("Goal_Pose: ",pose_goal)
        # pose_goal.orientation = pose.orientation
        move_group_arm.set_pose_target(pose_goal)
        
        # Now, we call the planner to compute the plan and execute it.
        plan = move_group_arm.go(wait=True)
        # Calling stop() ensures that there is no residual movement
        move_group_arm.stop()
        # It is always good to clear your targets after planning with poses.
        move_group_arm.clear_pose_targets()

def JointSpace_Goal_Subscriber(joints_goal:JointState):
    joints_values = joints_goal.position
    
    curr_joint_values = move_group_arm.get_current_joint_values()
    print("Joint values: ", curr_joint_values)
    # for i in range(len(joints_values)):
    #     if(not joints_values[i]<-100):
    #         curr_joint_values[i]=joints_values[i]
    print("Joint values: ", joints_values)
    print(" ------------------------------------------------------ ")
    print(" ------------------------------------------------------ ")
    print(" ------------------------------------------------------ ")
    move_group_arm.set_joint_value_target(joints_values)
    # Now, we call the planner to compute the plan and execute it.
    plan = move_group_arm.go(wait=True)
    # Calling stop() ensures that there is no residual movement
    move_group_arm.stop()
    # It is always good to clear your targets after planning with poses.
    move_group_arm.clear_pose_targets()

def Gripper_Control_Subscriber(Open_flag:Bool):
    open:bool = Open_flag.data 
    # move_group_gripper.ge
    joint_values = move_group_gripper.get_current_joint_values()
    if(open):
        joint_values[0]=-0.01
    else:
        joint_values[0]= 0.027
    move_group_gripper.set_joint_value_target(joint_values)
    # Now, we call the planner to compute the plan and execute it.
    plan = move_group_gripper.go(wait=True)
    # Calling stop() ensures that there is no residual movement
    move_group_gripper.stop()
    # It is always good to clear your targets after planning with poses.
    move_group_gripper.clear_pose_targets()

# Initialize the move_group API
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Manipulator_Controller_Node', anonymous=True)

# Instantiate a RobotCommander object. Provides information about the robot's kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object. Provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case, it is the manipulator
group_name = "arm"
move_group_arm = moveit_commander.MoveGroupCommander(group_name)

group_name = "gripper"
move_group_gripper = moveit_commander.MoveGroupCommander(group_name)


# We can get the name of the reference frame for this robot:
planning_frame = move_group_arm.get_planning_frame()
print("============ Reference frame: %s" % planning_frame)

# We can also print the name of the end-effector link for this group:
eef_link = move_group_arm.get_end_effector_link()
print("============ End effector: %s" % eef_link)

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Robot Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the robot:
print("============ Printing robot state")
print(robot.get_current_state())
print("")

# Planning to a Pose Goal
# pose_goal = Pose()


# print(move_group.get_joint_value_target())

# move_group.set_joint_value_target([0,-1,0.3,0.7])



sub = rospy.Subscriber("Manipulator_TaskSpace_Goal",Pose,TaskSpace_Goal_Subscriber)
sub2 = rospy.Subscriber("Manipulator_JointSpace_Goal",JointState,JointSpace_Goal_Subscriber)
sub3 = rospy.Subscriber("Manipulator_Gripper_Open_Close",Bool,Gripper_Control_Subscriber)


rate = rospy.Rate(10)  # 10 Hz

print("New Version of Code")
while not rospy.is_shutdown():
    rate.sleep()


