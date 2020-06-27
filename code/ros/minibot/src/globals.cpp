/*
 * globals.cpp
 *
 *  Created on: Jun 27, 2020
 *      Author: jochen
 */


#include "globals.h"

namespace Minibot {

// publisher of new joint values
ros::Publisher pub_joint_state_uiui;

// publisher of possible configurations
ros::Publisher pub_joint_values_config;

// publisher for new tcp data
ros::Publisher pub_tcp_ui;

// publisher for new joint_states
ros::Publisher pub_joint_state_ui;

// publisher for joint_states
ros::Publisher pub_joint_state;

// publisher for messages
ros::Publisher pub_msg;

// publisher for global trajectories
ros::Publisher pub_global_plan;

// publisher for local trajectories
ros::Publisher pub_local_plan;



minibot::Configuration settings;
minibot::PoseStorage pose_store;
minibot::Programme programme_store;

// cache the joint names of the group "minibot_arm" as defined in SRDF
std::vector<std::string> minibot_arm_joint_names; 			// joint names of the arm without the gripper
std::vector<std::string> minibot_gripper_joint_names;		// joint names of the gripper without the arm
std::vector<std::string> minibot_joint_names;				// all joint names, including gripper

minibot::GlobalPlan global_plan;



}
