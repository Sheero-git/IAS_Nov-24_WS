#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import math

from package.msg import Highcmd, HighState

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        
        self.high_cmd_pub = rospy.Publisher('/high_cmd', Highcmd, queue_size=10)
        rospy.Subscriber('/move_robot/target', Point, self.target_callback)
        rospy.Subscriber('/relative_point', Point, self.relative_point_callback)
        rospy.Subscriber('/high_state', HighState, self.high_state_callback)
        
        # Target and current position variables
        self.target_x = None
        self.target_y = None
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # State flags
        self.reached_target = False
        self.aligned_with_target = False

        # Velocity parameters
        self.max_vel = 0.65
        self.min_vel = 0.3
        self.tolerance = 0.05
        self.angular_tolerance = 0.03  # Small threshold to stop rotating
        self.rate = rospy.Rate(60)

        # Proportional-Derivative factors for angular velocity
        self.angular_velocity_p = 0.95
        self.angular_velocity_d = 0.1
        self.previous_angle_diff = 0.0
        self.linear_velocity = 0.2
        self.angular_velocity = 0.75

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.reached_target = False
        self.aligned_with_target = False
        rospy.loginfo(f"global target received: ({self.target_x}, {self.target_y})")

    def relative_point_callback(self, msg):
        relative_x = msg.x 
        relative_y = msg.y  
        self.target_x = self.current_x + (math.cos(self.current_yaw) * relative_x - math.sin(self.current_yaw) * relative_y)
        self.target_y = self.current_y + (math.sin(self.current_yaw) * relative_x + math.cos(self.current_yaw) * relative_y)
        self.reached_target = False
        self.aligned_with_target = False
        rospy.loginfo(f"relative point: ({self.target_x}, {self.target_y})")

    def high_state_callback(self, msg):
        """Update robot position and yaw from HighState message."""
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_yaw = msg.euler[2]  # Assuming yaw is the third value in euler angles

    def move_and_rotate_towards_target(self):
        if self.target_x is None or self.target_y is None:
            rospy.loginfo("waiting for target...")
            return

        # Calculate the target position in the robot's local coordinate frame
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        # Transform target coordinates to the robot's local frame
        local_x = math.cos(self.current_yaw) * dx + math.sin(self.current_yaw) * dy
        local_y = -math.sin(self.current_yaw) * dx + math.cos(self.current_yaw) * dy

        # Calculate the distance and angle to the target in the local frame
        distance = math.sqrt(local_x ** 2 + local_y ** 2)
        target_angle = math.atan2(local_y, local_x)

        rospy.loginfo(f"distance to target: {distance:.2f}, angle to target: {target_angle:.2f} rad")

        high_cmd_msg = Highcmd() # zy twist()
        if abs(target_angle) > self.angular_tolerance:
            d_angle = target_angle - self.previous_angle_diff
            high_cmd_msg.yawSpeed = max(
                min(self.angular_velocity_p * target_angle + self.angular_velocity_d * d_angle, self.angular_velocity),
                -self.angular_velocity
            )
            self.previous_angle_diff = target_angle
        else:
            high_cmd_msg.yawSpeed = 0.0
            self.aligned_with_target = True

        # move forward if aligned and not yet at the target
        if self.aligned_with_target and distance > self.tolerance:
        #cutoff
            scaled_velocity = max(min(self.linear_velocity * (distance / 2.0), self.max_vel), self.min_vel)
            high_cmd_msg.velocity[0] = scaled_velocity  
            high_cmd_msg.velocity[1] = 0.0  
        else:
            high_cmd_msg.velocity[0] = 0.0
            high_cmd_msg.velocity[1] = 0.0
            if distance <= self.tolerance:
                self.reached_target = True
                rospy.loginfo("Reached target!")

        # Publish the Highcmd message
        self.high_cmd_pub.publish(high_cmd_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.current_x is None or self.current_y is None or self.current_yaw is None:
                rospy.loginfo("Waiting for HighState data...")
                self.rate.sleep()
                continue

            if not self.reached_target:
                self.move_and_rotate_towards_target()
            else:
                rospy.loginfo("Awaiting new target...")
                high_cmd_msg = Highcmd()
                high_cmd_msg.velocity = [0.0, 0.0]  # Stop movement
                high_cmd_msg.yawSpeed = 0.0
                self.high_cmd_pub.publish(high_cmd_msg)

            self.rate.sleep()

if __name__ == "__main__":
    mover = MoveRobot()
    mover.run()

