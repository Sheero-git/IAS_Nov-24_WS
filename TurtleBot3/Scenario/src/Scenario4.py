#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String, Int32MultiArray

phase = 1
goal_Reached = False
object_done = False

# Callback function for the /path_flag topic
def goal_flag_callback(msg):
    global goal_Reached
    rospy.loginfo(f"Received flag from GoalSender: {msg.data}")
    if msg.data and not object_done:
        #Publish a String command to manipulator
        order = "Pickup" if phase == 1 else "Place"
        rospy.loginfo(f"Publishing order to manipulator: {order}")
        manipulator_signal.publish(order)
        goal_Reached = True

# Callback function for the /manipulator_flag topic
def manipulator_flag_callback(msg):
    global phase
    global object_done
    rospy.loginfo(f"Received flag from manipulator: {msg.data}")
    if msg.data:
        object_done = True
        x=0
        y=0
        angle=0

# Function to publish coordinates to the /coordinates topic
def publish_coordinates(x, y, z):
    goal = Int32MultiArray()
    goal.data = [x, y, z]
    rospy.loginfo(f"Publishing coordinates: ({x}, {y}, {z})")
    coordinates.publish(goal)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('bot', anonymous=True)
        rate = rospy.Rate(5)  # Set the publishing rate to 5 Hz

        # Publishers
        coordinates = rospy.Publisher('/coordinate', Int32MultiArray, queue_size=1)
        manipulator_signal = rospy.Publisher('/manipulator_signal', String, queue_size=1)

        # Subscribers
        path_flag = rospy.Subscriber('/path_flag', Bool, goal_flag_callback)
        manipulator_flag = rospy.Subscriber('/manipulator_flag', Bool, manipulator_flag_callback)

        # Publish initial coordinates for testing
        rospy.loginfo("Node initialized. Publishing initial coordinates.")
        x=3
        y=3
        angle = 0

        while not rospy.is_shutdown():
            publish_coordinates(x, y, angle)
            rate.sleep()
        #Publish a String command to manipulator
        #order = "Place"
        #rospy.loginfo(f"Publishing order to manipulator: {order}")
        #manipulator_signal.publish(order)


        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

