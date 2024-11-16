#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from gazebo_msgs.msg import ModelStates
import math

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=False)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/move_robot/target', Point, self.target_callback)
        rospy.Subscriber('/relative_point', Point, self.relative_point_callback)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_position)

        self.target_x = None
        self.target_y = None
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        self.reached_target = False
        self.aligned_with_target = False

        self.max_vel = 0.55
        self.min_vel = 0.15
        self.tolerance = 0.05
        self.angular_tolerance = 0.05
        self.rate = rospy.Rate(60)

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
        rospy.loginfo(f"New global target received: ({self.target_x}, {self.target_y})")

    def relative_point_callback(self, msg):
        relative_x = msg.x
        relative_y = msg.y
        self.target_x = self.current_x + (math.cos(self.current_yaw) * relative_x - math.sin(self.current_yaw) * relative_y)
        self.target_y = self.current_y + (math.sin(self.current_yaw) * relative_x + math.cos(self.current_yaw) * relative_y)
        self.reached_target = False
        self.aligned_with_target = False
        rospy.loginfo(f"New relative target set at global coordinates: ({self.target_x}, {self.target_y})")

    def update_position(self, msg):
        # Find the index of the robot in ModelStates
        try:
            index = msg.name.index("go1_gazebo")  # Replace with your robot's name in Gazebo
            self.current_x = msg.pose[index].position.x
            self.current_y = msg.pose[index].position.y
            orientation_q = msg.pose[index].orientation
            _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)
        except ValueError:
            rospy.logwarn("Robot name not found in ModelStates")

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)

    def move_and_rotate_towards_target(self):
        if self.target_x is None or self.target_y is None:
            rospy.loginfo("Waiting for target...")
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        local_x = math.cos(self.current_yaw) * dx + math.sin(self.current_yaw) * dy
        local_y = -math.sin(self.current_yaw) * dx + math.cos(self.current_yaw) * dy

        distance = math.sqrt(local_x ** 2 + local_y ** 2)
        target_angle = math.atan2(local_y, local_x)

        rospy.loginfo(f"Distance to target: {distance:.2f} meters, Angle to target: {target_angle:.2f} radians")

        twist = Twist()

        if abs(target_angle) > self.angular_tolerance:
            twist.angular.z = max(
                min(self.angular_velocity_p * target_angle, self.angular_velocity),
                -self.angular_velocity
            )
        else:
            twist.angular.z = 0.0
            self.aligned_with_target = True

        if abs(target_angle) < math.pi / 4 and distance > self.tolerance:
            scaled_velocity = max(min(self.linear_velocity * (distance / 2.0), self.max_vel), self.min_vel)
            twist.linear.x = scaled_velocity
        else:
            twist.linear.x = 0.0
            if distance <= self.tolerance:
                self.reached_target = True
                rospy.loginfo("Reached target!")

        self.cmd_vel_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            if self.current_x is None or self.current_y is None or self.current_yaw is None:
                rospy.loginfo("Waiting for position data from ModelStates...")
                self.rate.sleep()
                continue

            if not self.reached_target:
                self.move_and_rotate_towards_target()
            else:
                rospy.loginfo("Awaiting new target...")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)

            self.rate.sleep()

if __name__ == "__main__":
    mover = MoveRobot()
    mover.run()
