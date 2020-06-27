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
#include "utils.h"
#include "marker.h"
#include "execution.h"

#include "minibot/ErrorCodes.h"

namespace Minibot {
namespace Execution {
#define LOG_NAME "execution"

moveit::planning_interface::MoveGroupInterface* move_group = NULL;
void construct() {
  ROS_INFO_STREAM_NAMED(LOG_NAME, "module execution init");

  move_group = new moveit::planning_interface::MoveGroupInterface(Minibot::minibot_group_name);
}

minibot::ErrorCodes  execute (const minibot::LocalPlan& local_plan) {

	// set the first position as starting pose
	minibot::MinibotState start_state;
	start_state.joint_state.name = local_plan.joint_trajectory.joint_names;
	start_state.joint_state.position = local_plan.joint_trajectory.points[0].positions;
	Minibot::Kinematics::setLastMinibotState(start_state);

	// the next main loop will publish this to /joint_states
	// which is necessary for moveit, otherwise execute will complain about a different starting state

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

