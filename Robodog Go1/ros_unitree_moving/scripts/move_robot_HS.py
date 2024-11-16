#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from unitree_legged_msgs.msg import HighState
import math

class MoveRobot:
    def __init__(self):
        rospy.init_node('move_robot', anonymous=True)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/move_robot/target', Point, self.target_callback)
        rospy.Subscriber('/relative_point', Point, self.relative_point_callback)
        self.odom_sub = rospy.Subscriber("high_state_position", HighState, self.update_position)

        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.reached_target = False
        self.aligned_with_target = False
        self.error_x=0.0
        self.error_y=0.0
        self.max_vel = 0.65
        self.min_vel = 0.2
        self.tolerance = 0.05
        self.angular_tolerance = 0.05
        self.rate = rospy.Rate(60)
        self.prev_target=Point();
        self.prev_target.x=0.0
        self.prev_target.y=0.0
        self.prev_target.z=0.0
        self.angular_velocity_p = 0.95
        self.angular_velocity_d = 0.1
        self.previous_angle_diff = 0.0
        self.linear_velocity = 0.2
        self.angular_velocity = 0.75

    def target_callback(self, msg):
        self.calculate_error()
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

    def calculate_error(self):
        self.error_x=self.current_x-self.target_x
        self.error_y=self.current_y-self.target_y

    def update_position(self, msg):
        self.current_x = msg.position[0]-self.error_x
        self.current_y = msg.position[1]-self.error_y
        self.current_yaw= msg.imu.quaternion[2]
        #orientation_q = msg.imu.quaternion[2]
        #, _, self.current_yaw = self.euler_from_quaternion(orientation_q)
    

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
                rospy.loginfo("Waiting for odometry data...")
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
