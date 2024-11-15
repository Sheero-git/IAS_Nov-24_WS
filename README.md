# IAS_Nov-24_WS
This repository documents our work on the turtlebot, Robodog and Franka Emica Panda Robots
Franka Emika Panda README file (Documentation)

NOTE: This documentation is specifically targeted towards a Ubuntu OS with ROS noetic
NOTE: Make sure your laptop has a speed of 1Gb/s for ethernet connections as else, the camera would not work

### 1. Go to the following link (https://frankaemika.github.io/docs/installation_linux.html#)

	NOTE: This is the official GitHub repository for the Franka Emika Panda which contains 	installation instructions for the Franka Control Interface which allows control of the 	robotic arm. If it is not found simply write "Franka Emika Panda Documentation" and you 	will find a website named "Franka Control Interface (FCI) Documentation."

### 2. Follow the instructions for the installation of the Franka Emika Panda packages on Linux which can be found in the following link (https://frankaemika.github.io/docs/installation_linux.html#) 
```
	## a. The first step is "Installing from the ROS repositories" where the commands can be found in the following link 
	(https://frankaemika.github.io/docs/installation_linux.html#installing-	from-the-ros-repositories)

	## b. The second step is "Building from source" where the commands can be found in the following link 
	(https://frankaemika.github.io/docs/installation_linux.html#building-from-source)

	## c. The third step is "Setting Up the Real-Time Kernel" where the commands can be found in the following link 
	(https://frankaemika.github.io/docs/installation_linux.html#setting-up-the-real-time-kernel)

	## d. To verify that the new kernel is in fact a real-time kernel
		i. Reboot the system 
                ``` sudo reboot ```
		ii. Press on "Advanced Options for Ubuntu" 
		iii. Choose the downloaded real-time kernel (It should have a -rt extension)
		iv. Execute the command in the terminal
        ''' uname -a '''
		v. The string "PREEMPT-RT" should appear as a result
```
	NOTE: If the string "PREEMPT-RT" does not appear, then your kernel is not a real-time kernel and hence, you must repeat Steps (2. c-d) or preferably repeat Step 2.

# 3. Clone the GitHub repo (https://github.tik.uni-stuttgart.de/st166949/hardware-interface-fm) into the source folder (.src) of your catkin workspace (cd catkin_ws/src/) to use University of Stuttgart's hardware abstraction layer for the Franka Emika Panda
```
	NOTE: Its name should be "hardware-interface-fm". Inside this folder, you will find a "panda_gazebo_physical" with a "src" file inside where Moveit and librealsense should be 	installed 
 	 cd hardware-interface-fm/panda_gazebo_physical/src

	## a. To install the hardware abstraction layer, follow the steps described in the document "hardware-interface-fm.pdf" inside the "hardware-interface-fm" folder. Additionally, you 	can find all the API commands to use to control the Franka Emika Panda Robot.	

# 4. Install Moveit for ROS-noetic in the "src" folder of the "panda_gazebo_physical" folder inside the "hardware-interface-fm" folder 
	 cd hardware-interface-fm/panda_gazebo_physical/src/
	## a. Follow the instructions present in this link exactly 
         https://moveit.ai/install/source/

# 5. Install the "ROS Wrapper for Intel® RealSense™ Devices" required for successful installation of "LibRealSense2 SDK". This is also known as "realsense-ros".
	
	## a. Follow the installation instructions in the following link
         https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?tab=readme-ov-file
	## b. You can use either method 1 or 2 however, method 1 was utilized in the context of this README file.
	
```
# 6. Install librealsense2 v2.50 in the "src" folder of the "panda_gazebo_physical" folder inside the "hardware-interface-fm" folder (cd hardware-interface-fm/panda_gazebo_physical/src/) . This version supports the D400 series FRAMOS cameras used in our hardware set up and is compatible with the ROS wrappers mentioned in Step 4. 
```
	## a. Go to the following link
 	 https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
  
	## b. Follow the "Install Dependencies" instructions exactly
	## c. Follow the "Install librealsense2" instructions with one exceptions to make sure that you install v2.50 and not v2.55 or the most recent version
		i. Exception: After Step 3 under the "Install librealsense2" instructions execute the command "git checkout v2.50.0" inside the "librealsense" root directory
	## d. Follow the "Building librealsense2 SDK" instructions exactly with one exception
		ii. Exception: Replace the final command 
 	 sudo make uninstall && make clean && make && sudo make install
  with the following command 
        sudo make uninstall && make clean && make && sudo make -j$(($(nproc)-1)) install
  if your CPU has more than one core to allow for parallel compilation.
```
	NOTE: After executing Steps 5-6, you can execute the command "roslaunch realsense2_camera rs_camera.launch" to make sure that the camera real-sense device is detected and works
	NOTE: Additionally, you can execute the command "realsense-viewer" to be able to see the current camera image on the real-sense viewer app
	NOTE: The command "realsense-viewer" can also be used to verify the librealsense version since it is typically apparent when opening the real-sense viewer app.

# 7. Test the connection to the Franka Emika Panda Robot using Franka's desktop app "Desk"

	## a. Go to the following link 
        https://frankaemika.github.io/docs/getting_started.html
	## b. Follow the steps to configure the network connection to the Franka Emika Panda
	NOTE: Ignore the "Setting Up the Network" section if the Franka's LAN hardware connections have already been established
	## c. After accessing the "Desk" app, unlock the joints (Right Hand Side of the "Desk" app's GUI). After executing this, the franka's lights should turn blue.
	## d. Then you can execute predefined motion trajectories from the execute button in the "Desk" GUI.

# 8.  Run your own code after connecting successfully using the "Desk" app (Step 7)
	## a. Unlock the joints from the "Desk" app
	## b. Activate FCI 
	## c. Execute the command "roslaunch rl_launch rl_physical.launch" to establish the connection to the Franka Emika Panda
		NOTE: Upon executing this command, RViz should open and the camera should be detected.
	## d. Now, you can move the robot directly from RViz or you can execute your own python code on another terminal using the commands in the "hardware-interface-fm.pdf" folder.
	## e. To test programmatic control of the robot, you can run the file "hw_interface_test.py" to test the execution of all the commands in the hardware-abstraction-layer API





