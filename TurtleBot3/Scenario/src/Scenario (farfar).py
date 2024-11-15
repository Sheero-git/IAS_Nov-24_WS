#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String, Int32MultiArray

# Global variable for phase
phase = 1
sent = False
reached_once = False

# Callback function for the /path_flag topic
def goal_flag_callback(msg):
    global phase ,reached_once
    global sent
    rospy.loginfo(f"Received flag from GoalSender: {msg.data}")
    rospy.loginfo(f"reached flag: {reached_once}")
    if msg.data and not reached_once:
        # Move On to next phase
        phase+=1
        sent=False
        reached_once=True
        

# Callback function for the /manipulator_flag topic
def manipulator_flag_callback(msg):
    global phase
    global sent
    rospy.loginfo(f"Received flag from manipulator: {msg.data}")
    if msg.data:
        phase+=1
        sent=False
        

# Function to publish coordinates to the /coordinates topic
def publish_coordinates(x, y, z):
    goal = Int32MultiArray()
    goal.data = [x, y, z]
    rospy.loginfo(f"Publishing coordinates: ({x}, {y}, {z})")
    coordinates.publish(goal)

if __name__ == '__main__':
    try:
        # Initialize the ROS node                reached_once=False

        rospy.init_node('bot', anonymous=True)
        rate = rospy.Rate(5)  # Set the publishing rate to 5 Hz

        # Publishers
        coordinates = rospy.Publisher('/coordinate', Int32MultiArray, queue_size=1)
        manipulator_signal = rospy.Publisher('/manipulator_signal', String, queue_size=1)

        # Subscribers
        path_flag = rospy.Subscriber('/path_flag', Bool, goal_flag_callback)
        manipulator_flag = rospy.Subscriber('/manipulator_flag', Bool, manipulator_flag_callback)

        # Publish initial coordinates for testing
        rospy.loginfo("Node initialized.")
        while(not rospy.is_shutdown()):
            if(phase==1):
                publish_coordinates(3, 3, 0)
            elif(phase==2 and not sent):
                reached_once=False
                order = "Pickup"
                rospy.loginfo(f"Publishing order to manipulator: {order}")
                manipulator_signal.publish(order)
                sent=True
            elif(phase==3):
                publish_coordinates(0, 0, 0)
            elif(phase==4 and not sent):
                order = "Place"
                rospy.loginfo(f"Publishing order to manipulator: {order}")
                manipulator_signal.publish(order)
                sent=True
            rospy.loginfo(f"Phase is: {phase}")
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

