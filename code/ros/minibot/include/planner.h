#ifndef MININBOT_PLANNER_H_
#define MININBOT_PLANNER_H_

#include "ros/ros.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <minibot/PlanningAction.h>
#include <minibot/Programme.h>
#include <minibot/LocalPlan.h>
#include <minibot/GlobalPlan.h>
#include <minibot/MinibotState.h>
#include <minibot/PoseStorage.h>

namespace Minibot {

namespace Planner{

	extern minibot::GlobalPlan global_plan;

	// call me before anything else
	void init();

	// handle incoming planning requests
	bool handlePlanningAction(minibot::PlanningAction::Request &req,
								minibot::PlanningAction::Response &res);


	// compile a programme and create a global plan consisting of local plans
	minibot::GlobalPlan createGlobalPlan (minibot::Programme & prog, const minibot::PoseStorage& poses);

	// compile a local plan with from start_index to end_index
	minibot::LocalPlan createLocalPlan (minibot::Programme & prog, const minibot::PoseStorage& poses, int start_index, int goal_index);

	// publish the planned directory
	void displayTrajectory(const minibot::GlobalPlan& global_plan, int preveous_last_index);

	// @brief undisplay the trajectory with the given name
	void undisplayTrajectory(const minibot::GlobalPlan& global_plan, std::string trajectory_name);


	// plan a point to point trajectory
	void planPTP(const minibot::MinibotState& start_pos,
		  	     const minibot::MinibotState& goal_pos,
			     trajectory_msgs::JointTrajectory &joint_trajectory,
			     double velocity_scaling_factor,
			     double acceleration_scaling_factor);
}
}

#endif
