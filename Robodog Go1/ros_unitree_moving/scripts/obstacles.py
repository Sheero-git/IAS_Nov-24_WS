#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseArray

class ObstaclePublisher:
    def __init__(self):
        rospy.init_node('obstacle_publisher', anonymous=False)
        
        self.excluded_models = ["ground_plane", "static_environment", "go1_gazebo"]
        
        # ROS Publishers and Subscribers
        self.obstacle_pub = rospy.Publisher('/obstacles', PoseArray, queue_size=10, latch=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=10)
        
        # Set a controlled rate for publishing (e.g., 20 Hz)
        self.publish_rate = rospy.Rate(1000)  # 20 Hz

    def model_states_callback(self, msg):
        obstacles = PoseArray()
        obstacles.header.stamp = rospy.Time.now()
        obstacles.header.frame_id = "map"

        for i, name in enumerate(msg.name):
            if name in self.excluded_models:
                continue

            obstacle_pose = Pose()
            obstacle_pose.position = msg.pose[i].position
            obstacle_pose.orientation = msg.pose[i].orientation
            obstacles.poses.append(obstacle_pose)

        # Publish the full updated list of obstacles at the controlled rate
        self.obstacle_pub.publish(obstacles)
        rospy.loginfo(f"Published {len(obstacles.poses)} obstacles.")
        
        # Control the publish rate
        self.publish_rate.sleep()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    obstacle_publisher = ObstaclePublisher()
    obstacle_publisher.run()
