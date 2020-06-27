/*
 * constants.h
 *
 *  Created on: Jun 16, 2020
 *      Author: Jochen Alt
 */

#ifndef INCLUDE_CONSTANTS_H_
#define INCLUDE_CONSTANTS_H_

#include "ros/ros.h"

namespace Minibot {

const int joint_state_publish_rate = 25;	  					// [Hz] frequency of topic /joint_states
const double trajectory_sampling_time = 0.04; 					// [s] sample time to be used for trajectory generation (25Hz)

const double max_trans_vel= 1;									// [m/s] max translational velocity (used for cartesian path planning)
const double max_trans_acc =  2.25;								// [m/s²] max translational acceleration (used for cartesian path planning)
const double max_trans_dec = -5;								// [m/s²] max translational deceleration (used for cartesian path planning)
const double max_rot_vel = 1.57;

const std::string global_trajectory_name = "global_trajectory";	// marker name of global trajectory
const std::string local_trajectory_name= "local_trajectory";	// marker name of local trajectory

const std::string err_msg_prefix = "ERR:";						// prefix for error messages (UI turns red)
const std::string warn_msg_prefix = "WARN:";					// prefix for warnings (UI turns amber)
const std::string info_msg_prefix ="INFO:";						// prefix for info messages (UI turns green)

const std::string planner_prefix = "PLANNER:";					// prefix used in messages sent to the programme panel
const std::string kinematics_prefix = "KINEMATICS:";			// prefix used in messages sent to the kinematics panel
const std::string posestore_prefix = "POSESTORE:";				// prefix used in messages sent to the pose storage panel
const std::string settings_prefix = "SETTINGS:";				// prefix used in messages sent to the settings panel

}

#endif /* INCLUDE_CONSTANTS_H_ */
