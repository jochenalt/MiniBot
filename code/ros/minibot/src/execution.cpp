/*
 * execution.cpp
 *
 *  Created on: Jun 22, 2020
 *      Author: jochen
 */




#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "planner.h"
#include "kinematics.h"
#include "database.h"
#include "node.h"
#include "utils.h"
#include "marker.h"
#include "execution.h"

#include "minibot/ErrorCodes.h"

namespace Minibot {
namespace Execution {
#define LOG_NAME "execution"

moveit::planning_interface::MoveGroupInterface* move_group = NULL;
void init() {
  ROS_INFO_STREAM_NAMED(LOG_NAME, "module execution init");

  move_group = new moveit::planning_interface::MoveGroupInterface(Minibot::minibot_group_name);
}

minibot::ErrorCodes  execute (const minibot::LocalPlan& local_plan) {
	// robot_state::RobotStatePtr start_state_ptr = move_group->getCurrentState();
	// robot_state::RobotState start_state = *start_state_ptr;
	move_group->setStartStateToCurrentState();
	moveit_msgs::RobotTrajectory trajectory;
	trajectory.joint_trajectory = local_plan.joint_trajectory;

	moveit::planning_interface::MoveItErrorCode moveit_error_code = move_group->execute(trajectory);
	minibot::ErrorCodes error_code;
	if (moveit_error_code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOG_NAME, "execution returned moveit error " << moveit_error_code.val);
		error_code.val = minibot::ErrorCodes::FAILURE;
	} else {
		error_code.val = minibot::ErrorCodes::SUCCESS;

	}
	return error_code;

}

}
}

