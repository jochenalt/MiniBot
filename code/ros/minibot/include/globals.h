/*
 * globals.h
 *
 *  Created on: Jun 27, 2020
 *      Author: jochen
 */

#ifndef INCLUDE_GLOBALS_H_
#define INCLUDE_GLOBALS_H_

#include "ros/ros.h"
#include "database.h"
#include "planner.h"

namespace Minibot {

extern minibot::Configuration settings;
extern minibot::PoseStorage pose_store;
extern minibot::Programme programme_store;

// the following variables are defined in init()
extern std::vector<std::string> minibot_arm_joint_names; 		// joint names of the arm without the gripper
extern std::vector<std::string> minibot_gripper_joint_names;	// joint names of the gripper without the arm
extern std::vector<std::string> minibot_joint_names;			// all joint names, including gripper

// publisher of new joint values
extern ros::Publisher pub_joint_state_ui;

// publisher for joint_states
extern ros::Publisher pub_joint_state;

// publisher of possible configurations
extern ros::Publisher pub_joint_values_config;

// publish messages to UI
extern ros::Publisher pub_msg;

// publish new tcp to UI
extern ros::Publisher pub_tcp_ui;

// publish updates from gearwheel
extern ros::Publisher pub_gearwheel_pose;

// publisher for messages
extern ros::Publisher pub_msg;


// publisher for global trajectories
extern ros::Publisher pub_global_plan;

// publisher for local trajectories
extern ros::Publisher pub_local_plan;

extern minibot::GlobalPlan global_plan;
}

#endif /* INCLUDE_GLOBALS_H_ */
