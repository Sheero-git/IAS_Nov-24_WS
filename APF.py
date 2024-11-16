#!/usr/bin/env python3 
import numpy as np
import random
import math
import copy
import rospy
from geometry_msgs.msg import Twist   #import msg data type "Twist" to be published
from std_msgs.msg import String,Int32,Float64MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose      #import msg data type "Pose" to be subscribed
from nav_msgs.msg import Odometry
import numpy as np                    #import numpy for trignometric function, arrays... etc
from tf.transformations import euler_from_quaternion
import sys    
                      #import sys for extracting input from termminal (input from user)
#########################################################################################################



def DistancetoGoal():
    global Goal
    return DistancetoPoint(Goal)

def DistancetoPoint(point):
    global Current_pos
    dx = point[0] - Current_pos[0]
    dy = point[1] - Current_pos[1]
    return Mag([dx, dy])



def getAttractiveForce():
    global Current_pos
    global Goal
    global Katt
    global MaxAttractiveforce
    Fatt=[0,0]
    Fatt[0] = Katt*(Goal[0] - Current_pos[0])

    Fatt[1] = Katt*(Goal[1] - Current_pos[1])
    m=Mag(Fatt)
    if (m>MaxAttractiveforce):
        Fatt[0] = Fatt[0] * MaxAttractiveforce/m
        Fatt[1] = Fatt[1] * MaxAttractiveforce/m

    print("Forces Attractive: ", Fatt)

    return Fatt

def bra7tak(Xobs,Yobs,qstar,Dobs):

    if Dobs < qstar :
        frep_x = -Krep * (1 / Dobs - 1 / qstar) * -(Current_pos[0] - Xobs) / ((Dobs)**3)
        frep_y = -Krep * (1 / Dobs - 1 / qstar) * -(Current_pos[1] - Yobs) / ((Dobs)**3)
    else :
        frep_x = 0
        frep_y = 0

    return [frep_x, frep_y]

def getRepulsiveForce():
    global Obstacles
    global ObstacleSize
    global Rsafe
    FrTotal=[0,0]
    for i in range(len(Obstacles)):
        Obstacle = Obstacles[i]
        #print(i)
        qstar = Rsafe
        xobs = Obstacle[0]
        yobs = Obstacle[1]
        dobs = DistancetoPoint(Obstacle) - ObstacleSize
        Frep = bra7tak(xobs, yobs, qstar, dobs)
        FrTotal[0] += Frep[0]
        FrTotal[1] += Frep[1]
    
    print("Forces Repulsive: ", FrTotal)

    return FrTotal

def Mag(V):
    return math.sqrt(V[0]**2+V[1]**2)

def callback(data):
    global Current_pos	#Identify msg variable created as global variable
    global flag_initial
    msg = data ##Extract the data sent as message
    Current_pos[0] = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
    Current_pos[1] = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    Current_pos[2] = round(yaw, 4)
    flag_initial = 1				#Set flag to one
	  #print("sub" + str(position[0]))

def callbackGoal(data):
    global Goal	#Identify msg variable created as global variable
    global flag_initial
    msg = data.data ##Extract the data sent as message
    Goal[0] = msg[0]		#Round the value of x to 4 decimal places
    Goal[1] = msg[1]	#Round the value of y to 4 decimal places
    Goal[2] = msg[2]	#Round the value of theta to 4 decimal places
    flag_initial = 1				#Set flag to one
	  #print("sub" + str(position[0]))
def callbackObstacles(data):
    global Dynamic_Obstacles
    global Rsafe
    Dynamic_Obstacles.clear()
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    ranges = data.ranges
    x=np.arange(angle_min,angle_max,angle_increment)
    for i in range(len(x)-1):
        angle = x[i]
        angle = angle
        if angle > 2*math.pi:
            angle = angle - 2*math.pi
        if ranges[i] < Rsafe and ranges[i] != 0 and  (angle < 60*math.pi/180  or angle > 300*math.pi/180 ):
            temp = [[Current_pos[0] + ranges[i] * math.cos(angle - Current_pos[2]), Current_pos[1] + ranges[i] * math.sin(angle - Current_pos[2])]]
            Dynamic_Obstacles = Dynamic_Obstacles + temp
       # print("Dobs,",Dynamic_Obstacles)
       # print(Current_pos)

         

def RefreshObstacles():
    global Obstacles
    # global Static_Obstacles
    global Dynamic_Obstacles
    Obstacles = Dynamic_Obstacles#Static_Obstacles+Dynamic_Obstacles
    #print(Obstacles)




rospy.init_node('APF', anonymous=True) #Identify ROS Node
pub = rospy.Publisher('/Desired_Position', Float64MultiArray, queue_size=1) #Identify the publisher "pub1" to publish on topic "/Tut3_Control_Input" to send message of type "Twist"
rate = rospy.Rate(5) # rate of publishing msg 10hz
sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
sub3 = rospy.Subscriber('/Goal', Float64MultiArray, callbackGoal) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
sub4 =  rospy.Subscriber('/scan', LaserScan, callbackObstacles)

Katt = 2
Krep = 0.4
Rsafe = 0.5
MaxAttractiveforce = 20
maxRepulsiveForce = 10

Goal = [0, 0, 0]
Start = [0, 0, 0]
maxDist = 1 # define how clearly the map is scanned]
minDist = 0.2
NumberOfPoints=10
# Static_Obstacles=[[-1.72307692 ,-1.95487805],[-1.53076923 ,-1.95487805] ,[-1.33846154, -1.95487805], [-1.14615385, -1.95487805], [-0.95384615, -1.95487805], [-0.76153846, -1.95487805], [-0.56923077, -1.95487805], [-0.37692308, -1.95487805], [-1.72307692, -1.71097561], [-0.37692308, -1.71097561], [-0.18461538, -1.71097561], [-0.18461538, -1.46707317], [-1.72307692, -1.22317073], [-0.18461538, -1.22317073], [-1.72307692, -0.97926829], [-0.18461538, -0.97926829], [-1.72307692, -0.73536585],[-0.37692308, -0.73536585], [-0.18461538, -0.73536585], [-1.72307692, -0.49146341],[-0.37692308, -0.49146341], [-0.18461538, -0.49146341], [-0.37692308, -0.24756098], [-0.18461538, -0.24756098], [-0.37692308, -0.00365854], [-0.18461538, -0.00365854], [-0.37692308,  0.2402439 ], [-0.37692308,  0.48414634], [-1.91538462,  0.72804878], [-0.37692308,  0.72804878], [-1.91538462,  0.97195122], [-0.37692308,  0.97195122], [-1.91538462,  1.21585366], [-1.72307692 , 1.21585366], [-1.53076923,  1.21585366], [-1.33846154,  1.21585366], [-1.14615385,  1.21585366], [-0.37692308  ,1.21585366], [-1.53076923 , 1.4597561 ], [-1.33846154,  1.4597561 ], [-1.14615385,  1.4597561 ], [-0.95384615,  1.4597561 ], [-0.76153846,  1.4597561 ], [-0.56923077,  1.4597561 ], [-0.37692308,  1.4597561 ]]
Dynamic_Obstacles = [[0, 0]]
Obstacles = [[0, 0]]# pos of each obstacle x y 
ObstacleSize = 0.1 # size of each var
Current_pos = copy.deepcopy(Start)
Desired_pos = copy.deepcopy(Start)
Ftotal = [0, 0]
Distance = DistancetoGoal()
data_to_send = Float64MultiArray()

while(1 and not rospy.is_shutdown()):
    RefreshObstacles()
    print("Obstacles: ", Obstacles)
    Fa = getAttractiveForce()
    Fr = getRepulsiveForce()
    #Fr=[0,0]
    #print(Fr)
    Ftotal[0] = Fa[0] + Fr[0]
    Ftotal[1] = Fa[1] + Fr[1]
    m=Mag(Ftotal)

    if m>maxDist:
        Ftotal[0] = Ftotal[0] * maxDist/m
        Ftotal[1] = Ftotal[1] * maxDist/m
    
    
    Desired_pos[0] = Current_pos[0] + Ftotal[0]
    Desired_pos[1] = Current_pos[1] + Ftotal[1]

    if Distance < minDist:
        Desired_pos[0] = Goal[0]
        Desired_pos[1] = Goal[1]
    Desired_pos[2] = Goal[2]    
    if Fr[0] == 0 and Fr[1] == 0:
        Desired_pos[0] = Goal[0]
        Desired_pos[1] = Goal[1]

    data_to_send.data = Desired_pos
    print("DesiredPoint: ",data_to_send.data)
    print("Forces Total: ", Ftotal)
    pub.publish(data_to_send)
    #print(Current_pos)
    Distance=DistancetoGoal()
    #print(Distance) 
    rate.sleep()		#Sleep with rate  
