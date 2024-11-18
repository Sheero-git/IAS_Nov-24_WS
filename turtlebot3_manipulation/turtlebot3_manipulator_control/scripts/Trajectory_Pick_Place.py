#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from manipulator_control.srv import Gripper_Control_service



# Taskspace_Goal_Publisher = rospy.Publisher('/Manipulator_TaskSpace_Goal', Pose, queue_size=10)
Jointspace_Goal_Publisher = rospy.Publisher('/Manipulator_JointSpace_Goal', JointState, queue_size=10)
Gripper_Publisher = rospy.Publisher('/Manipulator_Gripper_Open_Close', Bool, queue_size=10)


# def Taskspace_Goal_callback(open_flag:Bool):
#     Taskspace_Goal_Publisher.publish(open_flag)  

def Jointspace_Goal_callback(open_flag:Bool):
    Jointspace_Goal_Publisher.publish(open_flag)  

def gripper_control_callback(open_flag:Bool):
    Gripper_Publisher.publish(open_flag)  



gripper_control_service = rospy.Service('Gripper_open',Gripper_Control_service,gripper_control_callback)

gripper_control_service = rospy.Service('Gripper_open',Gripper_Control_service,gripper_control_callback)

gripper_control_service = rospy.Service('Gripper_open',Gripper_Control_service,gripper_control_callback)


rospy.init_node('Manipulator_Pick_Place_Node', anonymous=True)
rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():

    rate.sleep()