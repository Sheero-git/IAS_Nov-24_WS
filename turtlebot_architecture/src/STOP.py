#!/usr/bin/env python3
# A basic python code to implement the kinematics model for the differential mobile robot
# and reflect its behavior on graphs.

#########################################################################################################
#Import the required libraries:
import rospy
from geometry_msgs.msg import Twist   #import msg data type "Twist" to be published
from geometry_msgs.msg import Pose      #import msg data type "Pose" to be subscribed
from nav_msgs.msg import Odometry
import numpy as np                    #import numpy for trignometric function, arrays... etc
from tf.transformations import euler_from_quaternion
import sys    
from std_msgs.msg import String,Int32,Float64MultiArray
import math                        #import sys for extracting input from termminal (input from user)
#########################################################################################################

def transformation():
    global rho		#Identify rho variable as global variable
    global beta		#Identify beta variable as global variable
    global alpha	#Identify alpha variable as global variable
    global x_des
    global y_des
    global theta_des
    global x
    global y
    global theta
    tolerance = 1/100
    x_delta = x_des - x		#Calculate the change in X direction
    y_delta = y_des - y	#Calculate the change in Y direction



    #Calculate distance rho representing relative distance between the desired and the current position
    rho = round(np.sqrt((np.square(x_delta))+ (np.square(y_delta))),3)

    #Calculate angle gamma representing the angle between the global X-direction of the vehicle and rho
    gamma = round(np.arctan2(y_delta,x_delta),3)		
    
    #while (0>gamma):
    #    gamma = gamma+2*math.pi
    while (gamma>2*math.pi):
        gamma = gamma-2*math.pi

    #Calculate angle alpha between the local X-direction of the vehicle and rho 
    alpha = gamma - theta
    print("alpha: ",alpha)


    #Calculate angle beta between rho and desired global X-direction of the vehicle
    beta = - alpha - theta + (theta_des)
    print("beta: ",beta)
#########################################################################################################
def control():
    global rho		#Identify rho variable as global variable
    global beta		#Identify beta variable as global variable 
    global alpha		#Identify alpha variable as global variable
    global linear_v	#Identify linear_v variable as global variable
    global angular_v	#Identify angular_v variable as global variable 
    global Krho
    global Kalpha
    global Kbeta  
    MaxV=0.26
    MaxW=1
    tolerance=1/100
   #Calculate controlled linear velocity and angular velocity
#Condition handles if desired position is infornt or behind the vehicle
    linear_v = Krho * rho
    linear_v = max(-MaxV, linear_v)
    linear_v = min(MaxV, linear_v)
    if(rho <tolerance):
        alpha=0

    angular_v = Kalpha * alpha + Kbeta*beta
    #if (rho <0.01):
    #    angular_v = Kbeta * beta
    #    angular_v = Kalpha * alpha
    #angular_v = max(-MaxW, angular_v)
    #angular_v = min(MaxW, angular_v)

def callback(data):
    global x	#Identify msg variable created as global variable
    global y	#Identify a subscriber as global variable
    global theta
	  #Identify all variables as globals 
    global flag_initial
    msg = data ##Extract the data sent as message
    x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
    y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    theta = round(yaw,4)
    flag_initial = 1				#Set flag to one
	  #print("sub" + str(position[0]))
def callbackPath(data):
    global x_des
    global y_des
    global theta_des
    A=data.data
    x_des = A[0]
    y_des = A[1]
    theta_des=A[2]

    
###########################################################################################################

rospy.init_node('Point_to_Point_Control', anonymous=True) #Identify ROS Node
pub1 = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Identify the publisher "pub1" to publish on topic "/Tut3_Control_Input" to send message of type "Twist"
vel_msg = Twist() #Identify msg variable of data type Twist
rate = rospy.Rate(10) # rate of publishing msg 10hz
sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
sub3=rospy.Subscriber('/Desired_Position', Float64MultiArray, callbackPath)


x_des = 6
y_des = 2
#theta_des = 90*math.pi/180
theta_des=0
x=0
y=0
theta=0
linear_v=0	#Identify linear_v variable as global variable
angular_v=0
#Initialize the parameters for coordinate transformation
rho = 0 #Initialization of variable rho
beta = 0 #Initialization of variable beta
alpha = 0 #Initialization of variable alpha
#######################################################################
#######################################################################
#Initialize the control gains
Krho = 0.5
Kalpha = 1.5
Kbeta = -0.7



#Krho   = 0.2
#Kalpha = 0.7
#Kbeta  = -3





####################################################################################################################################################################





while 1 and not rospy.is_shutdown():

    #Set the values of the Twist msg to be published
    vel_msg.linear.x = 0  #Linear Velocity
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0  #Angular Velocity

    #ROS Code Publisher
    
    pub1.publish(vel_msg)	#Publish msg
    
    
    rate.sleep()		#Sleep with rate
