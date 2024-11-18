import rospy
from geometry_msgs.msg import Pose
import numpy as np
from hw_interface import HwInterface  # Assume the HwInterface class is in a file named hw_interface.py
import matplotlib.pyplot as plt

# Initialize the hardware interface with panda, camera, and gazebo enabled
hw_interface = HwInterface(panda=True, cameras=["/image_raw"], gazebo=True)

#1. Define positions
#a. Home position
x_home=0.5; 
y_home=0;
z_home=0.3;
o_x_home=1; #Default quaternion corresponds to vertical position for vertical pick and place
o_y_home=0;
o_z_home=0;
o_w_home=0;

#b. 1st pick position (left looking at franka robot)
x_pick_1=0.5;
y_pick_1=-0.2;
z_pick_1=0.02;

#c. 2nd pick position (center looing at franka robot)
x_pick_2=0.5;
y_pick_2=0;
z_pick_2=0.02;

#d. 3rd pick position (right looking at franka robot)
x_pick_3=0.5;
y_pick_3=0.2;
z_pick_3=0.02;

#e. place position (pyramid)
x_place=0.4;
y_place=0;
z_place=0.02;

#f. Intermediate Coordinates
x_int=0.5;
y_int=-0.2;
z_int=0.5;

#f. Additional height for pyramid
dz_place_1=0.03;
dz_place_2=0.06;

#1. Start the motion
#a. Go to home position
success = hw_interface.move_cartesian(x=x_home, y=y_home, z=z_home, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)

#b. Perform 1st pick and place
#Go above pick position
success = hw_interface.move_cartesian(x=x_pick_1, y=y_pick_1, z=(z_pick_1+0.05), o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)
#Go to pick position
success = hw_interface.move_cartesian(x=x_pick_1, y=y_pick_1, z=z_pick_1, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Grasp object 
success = hw_interface.grasp_object(width=0.02, epsilon_inner=0.005, epsilon_outer=0.05, speed=0.1, force=5)
#Move to home
success = hw_interface.move_cartesian(x=x_home, y=y_home, z=z_home, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)
#Move to place location
success = hw_interface.move_cartesian(x=x_place, y=y_place, z=z_place , o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Release Object
success=hw_interface.move_gripper(width=0.5, speed=0.1)
#Move slow above place location
success = hw_interface.move_cartesian(x=x_place, y=y_place, z=(z_place+0.05), o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Move to home
success = hw_interface.move_cartesian(x=x_home, y=y_home, z=z_home, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)

#c. Perform 2nd pick and place
#Go above pick position
success = hw_interface.move_cartesian(x=x_pick_2, y=y_pick_2, z=(z_pick_2+0.05), o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)
#Go to pick position
success = hw_interface.move_cartesian(x=x_pick_2, y=y_pick_2, z=z_pick_2, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Grasp object 
success = hw_interface.grasp_object(width=0.02, epsilon_inner=0.005, epsilon_outer=0.05, speed=0.1, force=5)
#Move to home
success = hw_interface.move_cartesian(x=x_home, y=y_home, z=z_home, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)
#Move to place location
success = hw_interface.move_cartesian(x=x_place, y=y_place, z=z_place+dz_place_1 , o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Release Object
success=hw_interface.move_gripper(width=0.5, speed=0.1)
#Move slow above place location
success = hw_interface.move_cartesian(x=x_place, y=y_place, z=(z_place+dz_place_1+0.05), o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Move to home
success = hw_interface.move_cartesian(x=x_home, y=y_home, z=z_home, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)

#d. Perform 3rd pick and place
#Go above pick position
success = hw_interface.move_cartesian(x=x_pick_3, y=y_pick_3, z=(z_pick_3+0.05), o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)
#Go to pick position
success = hw_interface.move_cartesian(x=x_pick_3, y=y_pick_3, z=z_pick_3, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Grasp object 
success = hw_interface.grasp_object(width=0.02, epsilon_inner=0.005, epsilon_outer=0.05, speed=0.1, force=5)
#Move to home
success = hw_interface.move_cartesian(x=x_home, y=y_home, z=z_home, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)
#Move to place location
success = hw_interface.move_cartesian(x=x_place, y=y_place, z=z_place+dz_place_2 , o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)
#Release Object
success=hw_interface.move_gripper(width=0.5, speed=0.1)
#Move slow above place location
success = hw_interface.move_cartesian(x=x_place, y=y_place, z=(z_place+dz_place_2+0.05), o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.1, acc=0.1)

#Move to home
success = hw_interface.move_cartesian(x=x_home, y=y_home, z=z_home+0.2, o_x=o_x_home, o_y=o_y_home, o_z=o_z_home, o_w=o_w_home, vel=0.2, acc=0.15)

