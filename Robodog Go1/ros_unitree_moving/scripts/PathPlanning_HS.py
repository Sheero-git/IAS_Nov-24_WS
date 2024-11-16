#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, PoseArray
import math
from unitree_legged_msgs.msg import HighState
class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=False)
        
        self.goal = None
        self.current_position = Point()
        self.current_yaw = 0.0  # Yaw angle of the robot
        self.k_att = 2.0
        self.k_rep = 5.0
        self.repulsion_radius = 1
        self.tolerance = 0.05
        self.obstacles = []

        # ROS Publishers and Subscribers
        self.target_pub = rospy.Publisher('move_robot/target', Point, queue_size=10)
        self.goal_sub = rospy.Subscriber('move_robot/goal', Point, self.set_goal)
        self.relative_goal_sub = rospy.Subscriber('move_robot/relative_goal', Point, self.relative_goal_callback)
        self.odom_sub = rospy.Subscriber("high_state_position", HighState, self.update_position)
        self.obstacles_sub = rospy.Subscriber('/obstacles', PoseArray, self.obstacles_callback)
        
        self.rate = rospy.Rate(10)

    def set_goal(self, goal_msg):
        self.goal = goal_msg
        rospy.loginfo(f"New goal set at: ({self.goal.x}, {self.goal.y})")
        self.plan_path()

    def relative_goal_callback(self, msg):
        relative_x = msg.x
        relative_y = msg.y
        global_goal_x = self.current_position.x + (math.cos(self.current_yaw) * relative_x - math.sin(self.current_yaw) * relative_y)
        global_goal_y = self.current_position.y + (math.sin(self.current_yaw) * relative_x + math.cos(self.current_yaw) * relative_y)
        self.goal = Point(global_goal_x, global_goal_y, 0.0)
        rospy.loginfo(f"New relative goal set at global coordinates: ({self.goal.x}, {self.goal.y})")
        self.plan_path()

    def update_position(self, msg):
        self.current_position.x = msg.position[0]
        self.current_position.y = msg.position[1]
        self.current_yaw = msg.imu.quaternion[2]
        #orientation_q = msg.imu.quaternion[2]
        #_, _, self.current_yaw = self.euler_from_quaternion(orientation_q)

    def obstacles_callback(self, msg):
        self.obstacles = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return 0.0, 0.0, math.atan2(siny_cosp, cosy_cosp)

    def plan_path(self):
        if not self.goal:
            rospy.logwarn("No goal set.")
            return

        while not self.reached_goal():
            attractive_force = self.calculate_attractive_force(self.current_position, self.goal)
            repulsive_force = self.calculate_repulsive_force(self.current_position, self.obstacles)
            
            force_x = attractive_force[0] + repulsive_force[0]
            force_y = attractive_force[1] + repulsive_force[1]

            magnitude = math.sqrt(force_x**2 + force_y**2)
            if magnitude > 0:
                force_x, force_y = force_x / magnitude, force_y / magnitude

            next_x = self.current_position.x + 0.1 * force_x
            next_y = self.current_position.y + 0.1 * force_y

            self.publish_waypoint(next_x, next_y)
            self.wait_for_robot_reach_target(next_x, next_y)

    def calculate_attractive_force(self, current, goal):
        distance = math.sqrt((goal.x - current.x)**2 + (goal.y - current.y)**2)
        if distance < self.tolerance:
            return 0.0, 0.0

        force_x = self.k_att * (goal.x - current.x)
        force_y = self.k_att * (goal.y - current.y)
        return force_x, force_y

    def calculate_repulsive_force(self, current, obstacles):
        total_force_x = 0.0
        total_force_y = 0.0
        obstacles_temp=obstacles
        for ox, oy in obstacles_temp:
            distance = math.sqrt((ox - current.x)**2 + (oy - current.y)**2)
            if distance <= self.repulsion_radius:
                force_magnitude = self.k_rep * ((1.0 / distance) - (1.0 / self.repulsion_radius)) / (distance**2)
                force_x = force_magnitude * (current.x - ox)
                force_y = force_magnitude * (current.y - oy)
                total_force_x += force_x
                total_force_y += force_y

        return total_force_x, total_force_y

    def reached_goal(self):
        distance_to_goal = math.sqrt((self.goal.x - self.current_position.x)**2 + (self.goal.y - self.current_position.y)**2)
        return distance_to_goal < self.tolerance
        
            

    def publish_waypoint(self, x, y):
        target = Point()
        target.x = x
        target.y = y
        self.target_pub.publish(target)
        rospy.loginfo(f"Publishing waypoint: ({x}, {y})")

    def wait_for_robot_reach_target(self, target_x, target_y):
        while True:
            distance = math.sqrt((self.current_position.x - target_x)**2 + (self.current_position.y - target_y)**2)
            if distance < self.tolerance:
                break
            rospy.sleep(0.1)

if __name__ == '__main__':
    planner = PathPlanner()
    rospy.spin()
