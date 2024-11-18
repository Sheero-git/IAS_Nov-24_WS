#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <iostream>
#include <vector>
#include "../include/MiroSheesho/qnode.hpp"

/*
// Define global variables
MiroSheesho::QNode *qnode_ptr = nullptr;
double path_time = 4.0;
ros::Publisher manipulator_flag_pub; // Publisher for /manipulator_flag

// Callback function for /manipulator_signal
void manipulatorSignalCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("Received signal: %s", msg->data.c_str());
    std_msgs::Bool flag_msg;

    if (msg->data == "Pickup") {
        // 1. Open the gripper to pick up the object
        std::vector<double> gripper_open = {0.010};
        ROS_INFO("Opening gripper...");
        if (!qnode_ptr->setToolControl(gripper_open)) {
            ROS_ERROR("Failed to open gripper");
            return;
        }
        ros::Duration(1.0).sleep();

        // 2. Move to pick position
        std::vector<double> pick_position = {0.0, 1.2, -1, 1.5};
        ROS_INFO("Moving to pick position...");
        if (!qnode_ptr->setJointSpacePath(pick_position, path_time)) {
            ROS_ERROR("Failed to move to pick position");
            return;
        }
        ros::Duration(path_time).sleep();

        // 3. Close the gripper
        std::vector<double> gripper_close = {-0.010};
        ROS_INFO("Closing gripper...");
        if (!qnode_ptr->setToolControl(gripper_close)) {
            ROS_ERROR("Failed to close gripper");
            return;
        }
        ros::Duration(1.0).sleep();

        // 4. Move to home position
        //std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0};
        std::vector<double> home_position = {0.0, -1.0, 0.3, 0.7};
        ROS_INFO("Moving to home position...");
        if (!qnode_ptr->setJointSpacePath(home_position, path_time)) {
            ROS_ERROR("Failed to move to home position");
            return;
        }
        ros::Duration(path_time).sleep();
        ROS_INFO("Pickup operation completed.");

        // Publish flag indicating operation completion
        flag_msg.data = true;
        manipulator_flag_pub.publish(flag_msg);
    } 
    else if (msg->data == "Place") {
        // 1. Move to place position
        std::vector<double> place_position = {0.0, 1.2, -1, 1.5};
        ROS_INFO("Moving to place position...");
        if (!qnode_ptr->setJointSpacePath(place_position, path_time)) {
            ROS_ERROR("Failed to move to place position");
            return;
        }
        ros::Duration(path_time).sleep();

        // 2. Open the gripper to release the object
        std::vector<double> gripper_open = {0.010};
        ROS_INFO("Opening gripper...");
        if (!qnode_ptr->setToolControl(gripper_open)) {
            ROS_ERROR("Failed to open gripper");
            return;
        }
        ros::Duration(1.0).sleep();

        // 3. Move to home position
        //std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0};
        std::vector<double> home_position = {0.0, -1.0, 0.3, 0.7};

        ROS_INFO("Moving to home position...");
        if (!qnode_ptr->setJointSpacePath(home_position, path_time)) {
            ROS_ERROR("Failed to move to home position");
            return;
        }
        ros::Duration(path_time).sleep();
        ROS_INFO("Place operation completed.");

        // Publish flag indicating operation completion
        flag_msg.data = true;
        manipulator_flag_pub.publish(flag_msg);
    } 
    else {
        ROS_WARN("Unknown signal received.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_and_place_node");
    ros::NodeHandle nh;

    // Initialize QNode
    MiroSheesho::QNode qnode(argc, argv);
    qnode_ptr = &qnode;
    if (!qnode.init()) {
        ROS_ERROR("Failed to initialize QNode.");
        return -1;
    }
    ros::Duration(1.0).sleep();
    ROS_INFO("Node initialized successfully.");

    // Publisher for /manipulator_flag
    manipulator_flag_pub = nh.advertise<std_msgs::Bool>("/manipulator_flag", 10);

    // Subscriber for /manipulator_signal
    ros::Subscriber manipulator_signal_sub = nh.subscribe("/manipulator_signal", 10, manipulatorSignalCallback);

    //Move to home position
    std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0};
    ROS_INFO("Moving to home position...");
    if (!qnode_ptr->setJointSpacePath(home_position, path_time)) {
        ROS_ERROR("Failed to move to home position");
    }

    // Spin to process incoming messages
    //ros::spin();
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    ros::waitForShutdown(); // Wait until the node is shut down
    return 0;
}
*/

// Define global variables
MiroSheesho::QNode *qnode_ptr = nullptr;
double path_time = 4.0;
ros::Publisher manipulator_flag_pub; // Publisher for /manipulator_flag

// Callback function for /manipulator_signal
void manipulatorSignalCallback(const std_msgs::String::ConstPtr &msg) {
    ROS_INFO("Received signal: %s", msg->data.c_str());
    std_msgs::Bool flag_msg;

    if (msg->data == "Pickup") {
        // 1. Open the gripper to pick up the object
            std::vector<double> gripper_open = {0.010}; // Value to close the gripper
        ROS_INFO("Opening gripper...");
        if (!qnode.setToolControl(gripper_open)) {
            ROS_ERROR("Failed to open gripper");
            return -1;
        }
        ros::Duration(1.0).sleep(); // Allow time for gripper to close
        ROS_INFO("Gripper opened successfully.");
        // 2. Move to pick position
        std::vector<double> pick_position_kinematics = {0.144,0, 0.040}; // Example joint angles for pick position
        ROS_INFO("Moving to pick position: [%.2f, %.2f, %.2f]", pick_position_kinematics[0], pick_position_kinematics[1], pick_position_kinematics[2]);
        if (!qnode.setTaskSpacePath(pick_position_kinematics, path_time)) {
            ROS_ERROR("Failed to move to pick position");
            return -1;
        }
        ros::Duration(7).sleep();
        ROS_INFO("Successfully moved to pick position.");

        // 3. Close the gripper
        std::vector<double> gripper_close = {-0.010}; // Value to close the gripper
        ROS_INFO("Closing gripper...");
        if (!qnode.setToolControl(gripper_close)) {
            ROS_ERROR("Failed to close gripper");
            return -1;
        }
        ros::Duration(1.0).sleep(); // Allow time for gripper to close
        ROS_INFO("Gripper closed successfully.");
        // 4. Move to home position
        std::vector<double> ntr_position_kinematics = {0.194, 0.0, 0.304}; // Example joint angles for pick position
        ROS_INFO("Moving to home position: [%.2f, %.2f, %.2f]", ntr_position_kinematics[0], ntr_position_kinematics[1], ntr_position_kinematics[2]);
        if (!qnode.setTaskSpacePath(ntr_position_kinematics, path_time)) {
            ROS_ERROR("Failed to move to home position");
            return -1;
        }
        ros::Duration(path_time).sleep();
        ROS_INFO("Successfully moved to place position.");
        ros::Duration(2).sleep();

            // Publish flag indicating operation completion
            flag_msg.data = true;
            manipulator_flag_pub.publish(flag_msg);
        } 
        else if (msg->data == "Place") {
        // 1. Move to place position
        std::vector<double> pick_position_kinematics = {0.144,0, 0.040}; // Example joint angles for pick position
        ROS_INFO("Moving to pick position: [%.2f, %.2f, %.2f]", pick_position_kinematics[0], pick_position_kinematics[1], pick_position_kinematics[2]);
        if (!qnode.setTaskSpacePath(pick_position_kinematics, path_time)) {
            ROS_ERROR("Failed to move to pick position");
            return -1;
        }

        // 2. Open the gripper to release the object
        gripper_open = {0.010}; // Value to opewn the gripper
        ROS_INFO("Opening gripper...");
        if (!qnode.setToolControl(gripper_open)) {
            ROS_ERROR("Failed to open gripper");
            return -1;
        }
        ros::Duration(1.0).sleep(); // Allow time for gripper to close
        ROS_INFO("Gripper opened successfully.");

        // 3. Move to home position
        //std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0};
        ntr_position_kinematics = {0.194, 0.0, 0.304}; // Example joint angles for pick position
        ROS_INFO("Moving to place position: [%.2f, %.2f, %.2f]", ntr_position_kinematics[0], ntr_position_kinematics[1], ntr_position_kinematics[2]);
        if (!qnode.setTaskSpacePath(ntr_position_kinematics, path_time)) {
            ROS_ERROR("Failed to move to place position");
            return -1;
        }
        ros::Duration(path_time).sleep();
        ROS_INFO("Successfully moved to place position.");
        ros::Duration(2).sleep();

        // Publish flag indicating operation completion
        flag_msg.data = true;
        manipulator_flag_pub.publish(flag_msg);
    } 
    else {
        ROS_WARN("Unknown signal received.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_and_place_node");
    ros::NodeHandle nh;

    // Initialize QNode
    MiroSheesho::QNode qnode(argc, argv);
    qnode_ptr = &qnode;
     // Initialize node and wait for ROS to be ready
    if (!qnode.init()) {
        std::cerr << "Failed to initialize QNode." << std::endl;
        return -1;
    }
    ros::Duration(1.0).sleep();
    ROS_INFO("Node initialized successfully.");

    // Publisher for /manipulator_flag
    manipulator_flag_pub = nh.advertise<std_msgs::Bool>("/manipulator_flag", 10);

    // Subscriber for /manipulator_signal
    ros::Subscriber manipulator_signal_sub = nh.subscribe("/manipulator_signal", 10, manipulatorSignalCallback);

    //Move to home position
    ntr_position_kinematics = {0.194, 0.0, 0.304}; // Example joint angles for pick position
    ROS_INFO("Moving to place position: [%.2f, %.2f, %.2f]", ntr_position_kinematics[0], ntr_position_kinematics[1], ntr_position_kinematics[2]);
    if (!qnode.setTaskSpacePath(ntr_position_kinematics, path_time)) {
        ROS_ERROR("Failed to move to place position");
        return -1;
    }
    ros::Duration(path_time).sleep();
    ROS_INFO("Successfully moved to place position.");
    ros::Duration(2).sleep();

    // Spin to process incoming messages
    //ros::spin();
    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    ros::waitForShutdown(); // Wait until the node is shut down
    return 0;
}