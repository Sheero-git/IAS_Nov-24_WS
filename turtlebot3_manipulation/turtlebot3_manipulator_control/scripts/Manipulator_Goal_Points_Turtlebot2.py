#!/usr/bin/env python3

import sys
import rospy
import numpy as np
import moveit_commander
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64MultiArray
import time
import sensor_msgs

rospy.init_node('Turtlebot2/Manipulator',anonymous=True)
rate=rospy.Rate(10)
MQTT_data=''
Camera_data=''

# # Points for Robot 1 (94)

# Home=[0.0,0.0,0.0,0.0]
# Gripping_Position=[0.0,0.6,0.0,0.3]
# Red_Position=[0.0,0.6,0.0,0.3]
# Blue_Position=[0.5,0.6,0.0,0.3]
# White_Position=[-0.5,0.6,0.0,0.3]

# Points for Robot 2 (96)

Home=[0.0,0.0,0.0,0.0]
Gripping_Position=[3.14,-0.6,0.0,0.0]
Red_Position=[2.5,-0.6,0.0,0.0]
Blue_Position=[3.14,-0.6,0.0,0.0]
White_Position=[3.5,-0.6,0.0,0.0]


Grip_index=0
Release_index=0
Published_Points=JointState()


def Read_MQTT(data):
    global MQTT_data
    MQTT_data=data.data
    
sub_MQTT=rospy.Subscriber('/Turtlebot2/MQTT_msg',String,Read_MQTT)

def Read_Camera(data):
    global Camera_data
    Camera_data=data.data

sub_Camera=rospy.Subscriber('/Turtlebot2/Camera_msg',String,Read_Camera)

Gripping_Points=rospy.Publisher('/Turtlebot2/Manipulator_JointSpace_Goal',JointState,queue_size=1)
Gripping_Order=rospy.Publisher('/Turtlebot2/Manipulator_Gripper_Open_Close',Bool,queue_size=1)
Gripping_Status=rospy.Publisher('/Turtlebot2/Gripping_Status',String,queue_size=1)

Published_Points.position=Home
        # Go to Home
Gripping_Points.publish(Published_Points)
Gripping_Order.publish(False)
while not rospy.is_shutdown():
    #print(MQTT_data)
    if MQTT_data=="Grip_Now" and Grip_index==0:
        
        # Point where the object will be gripped
        Published_Points.position=Gripping_Position
        # Go to grip the object
        Gripping_Points.publish(Published_Points)
        print('Published')
        time.sleep(5)
        # If 96, false to grip. If 94, true to grip
        Gripping_Order.publish(True)
        time.sleep(1)
        # Home Point
        Published_Points.position=Home
        # Go to Home
        Gripping_Points.publish(Published_Points)
        time.sleep(5)
        Gripping_Status.publish("Gripped")
        time.sleep(1)
        Gripping_Status.publish('')
        Grip_index=1


    if MQTT_data=="Release" and Release_index==0:
        if Camera_data=="Red":
            #Points of Red position
            Published_Points.position=Red_Position
        elif Camera_data=="Blue":
            #Points of Blue position
            Published_Points.position=Blue_Position
        else:
            #Points of White position
            Published_Points.position=White_Position


        # Go to release the object
        Gripping_Points.publish(Published_Points)
        time.sleep(5)
        # If 96, true to release. If 94, false to release
        Gripping_Order.publish(False)
        time.sleep(1)
        # Home Point
        Published_Points.position=Home
        # Go to Home
        Gripping_Points.publish(Published_Points)
        time.sleep(5)
        Gripping_Status.publish("Released")
        time.sleep(1)
        Gripping_Status.publish('')
        Release_index=1