#include "planner.h"
#include "database.h"
#include "node.h"
#include "utils.h"
#include "marker.h"

#include <minibot/Programme.h>
#include <minibot/ErrorCodes.h>
#include <minibot/GlobalPlan.h>
#include <minibot/LocalPlan.h>
#include <minibot/Statement.h>

#include <trap_velocity_profile.h>

namespace Minibot {

#define LOG_NAME "planner"

std::string planner_prefix = "PLANNER:";

std::string global_trajectory_name = "global_trajectory";
std::string local_trajectory_name= "local_trajectory";

namespace Planner {

bool viz_global_plan = false;	// true, if the global plan is published and displayed
bool viz_local_plan = false;	// true if the local plan is published and displayed (defined by local_plan_index)
int local_plan_index = -1;		// index of the programme.statement that is to be displayed as local plan
minibot::GlobalPlan global_plan;

void init() {
	ROS_INFO_STREAM_NAMED(LOG_NAME, "module planner init");
}


minibot::GlobalPlan createGlobalPlan (minibot::Programme & prog, const minibot::PoseStorage& poses) {
	minibot::GlobalPlan global_plan;
	global_plan.error_code.val = minibot::ErrorCodes::SUCCESS;

	bool is_in_trajectory = false;
	int start_index = -1;
	int end_index = -1;
	int number_of_points = 0;
	for (int idx = 0;idx< prog.statements.size();idx++) {
		bool is_last = (idx == prog.statements.size()-1);
		minibot::Statement& stmt = prog.statements[idx];

		// think positive
		stmt.error_code.val = minibot::ErrorCodes::SUCCESS;

		if (is_in_trajectory) {
			// in a trajectory we wait for the next waypoint or movements
			if (stmt.type == minibot::Statement::STATEMENT_TYPE_WAYPOINT) {
				number_of_points ++;
				if (is_last) {
					number_of_points ++;
					end_index = idx;
				}
			}
			else {
				if (stmt.type == minibot::Statement::STATEMENT_TYPE_MOVEMENT) {
					number_of_points ++;
					end_index = idx;
				} else {
					// a trajectory ends at a wait statement
					if (stmt.type == minibot::Statement::STATEMENT_TYPE_WAIT) {
						end_index = idx;
					}
				}
			}
		} else {
			// outside a trajctory
			if (stmt.type == minibot::Statement::STATEMENT_TYPE_MOVEMENT) {
				start_index = idx;
				is_in_trajectory = true;
				number_of_points = 1;
			}
			else {
				if (stmt.type == minibot::Statement::STATEMENT_TYPE_WAYPOINT) {
					stmt.error_code.val = minibot::ErrorCodes::TRAJECTORY_CANNOT_START_WITH_WAYPOINT;
					global_plan.error_code.val = minibot::ErrorCodes::FAILURE;
				}
			}
		}

		// did we find a local plan?
		if ((start_index >= 0) && (end_index >=0) && (number_of_points > 1)) {
			minibot::LocalPlan local_plan = createLocalPlan(prog, poses, start_index, end_index);
			global_plan.local_plan.push_back(local_plan);
			start_index = -1;
			is_in_trajectory = false;
			end_index = -1;
			number_of_points = 0;
			if (local_plan.error_code.val != minibot::ErrorCodes::SUCCESS)
				global_plan.error_code.val = minibot::ErrorCodes::FAILURE;
		}
		if ((start_index >= 0) && (end_index >=0) && (number_of_points <= 1)) {
			stmt.error_code.val = minibot::ErrorCodes::LOCAL_TRAJECTORY_TOO_SHORT;
			global_plan.error_code.val = minibot::ErrorCodes::FAILURE;
		}
	}
	return global_plan;
}



minibot::LocalPlan createLocalPlan (minibot::Programme & prog, const minibot::PoseStorage& poses, int start_index, int goal_index) {
	minibot::LocalPlan local_plan;
	local_plan.start_index = start_index;
	local_plan.goal_index = goal_index;
    trajectory_msgs::JointTrajectory joint_trajectory;

    // gather start, end and waypoints
    std::vector<minibot::MinibotState> waypoints;
    for (size_t idx = start_index;idx < goal_index;idx++) {
    	minibot::Statement stmt = prog.statements[idx];
    	int pose_idx = Utils::findPose(poses, stmt.uid);
    	minibot::MinibotState state = poses.states[pose_idx];
    	if ((stmt.type == minibot::Statement::STATEMENT_TYPE_WAYPOINT) ||
    	    (stmt.type == minibot::Statement::STATEMENT_TYPE_MOVEMENT)) {
    		waypoints.push_back(state);
   	    }
    }

    switch (prog.statements[start_index].path_strategy) {
		case minibot::Statement::PLAN_SPACE_STRATEGY: {
			// SPACE strategy cannot deal with waypoints, it is just a concatenation of local plans
		    for (size_t idx = 0;idx < waypoints.size()-1;idx++) {
			    trajectory_msgs::JointTrajectory local_local_traj;
				Minibot::Planner::planPTP(waypoints[idx], waypoints[idx+1], local_local_traj, 0,0);
				Utils::append(joint_trajectory, local_local_traj);
		    }
		    local_plan.joint_trajectory = joint_trajectory;
			break;
		}
		default:
			break;
	}
	return local_plan;
}

void undisplayTrajectory(const minibot::GlobalPlan& global_plan, std::string trajectory_name) {

	int last_index = 0;
	if (Minibot::Planner::global_plan.local_plan.size()> 0)
		last_index = Minibot::Planner::global_plan.local_plan.back().start_index;

	// undisplay any old trajectories beyond the last one
	for (int traj_idx = 0; traj_idx <= last_index; traj_idx++)
		Minibot::Gearwheel::deleteTrajectoryMarker(trajectory_name, traj_idx);
}

void displayTrajectory(const minibot::GlobalPlan& global_plan, int preveous_last_index) {
	int old_traj_idx = 0; // index position from which old trajectories are deleted
	int color_no = 0;
	for (minibot::LocalPlan local_plan : global_plan.local_plan) {
		// undisplay old trajectories including the one that is about to be displayed (but maybe not)
		for (int old_traj_dx = old_traj_idx; old_traj_dx <local_plan.start_index; old_traj_dx++)
			Minibot::Gearwheel::deleteTrajectoryMarker(global_trajectory_name, old_traj_dx);
		old_traj_idx =local_plan.start_index;

		if (viz_global_plan) {
			Minibot::Gearwheel::createTrajectoryMarker(local_plan.joint_trajectory, true, global_trajectory_name, local_plan.start_index, color_no);
			pub_global_plan.publish(local_plan.joint_trajectory);
			old_traj_idx = local_plan.start_index + 1; // don't delete this trajectory in the next loop
		}
		else {
			if (viz_local_plan && local_plan.start_index <= local_plan_index && local_plan.goal_index >= local_plan_index ) {
				Minibot::Gearwheel::createTrajectoryMarker(local_plan.joint_trajectory, false, local_trajectory_name, local_plan.start_index, color_no);
				pub_local_plan.publish(local_plan.joint_trajectory);
				old_traj_idx = local_plan.start_index + 1;// don't delete this trajectory in the next loop
			}
		}
	}

	// undisplay any old trajectories beyond the last one
	for (int traj_dx = old_traj_idx; traj_dx <=preveous_last_index; traj_dx++)
		Minibot::Gearwheel::deleteTrajectoryMarker(global_trajectory_name, traj_dx);
}


// handle incoming planning requests
bool handlePlanningAction(minibot::PlanningAction::Request &req,
						  	minibot::PlanningAction::Response &res) {
	switch (req.type) {
		case minibot::PlanningAction::Request::FORWARD:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::BACKWARD:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::RUN:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::SELECT_LOCAL_PLAN:
			local_plan_index = req.start_index;
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::CLEAR_PLAN:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::GLOBAL_PLAN: {
			minibot::Programme prgm = Minibot::Database::getProgramme();
			minibot::PoseStorage poses= Minibot::Database::getPoseStorage();

			int last_index = 0;
			if (Minibot::Planner::global_plan.local_plan.size()> 0)
				last_index = Minibot::Planner::global_plan.local_plan.back().start_index;

			Minibot::Planner::global_plan = createGlobalPlan(prgm, poses);
			displayTrajectory(global_plan, last_index);
			res.error_code.val = global_plan.error_code.val;
			break;
		}
		case minibot::PlanningAction::Request::VIS_GLOBAL_PLAN: {
			if (viz_global_plan != req.jfdi) {
				viz_global_plan = req.jfdi;
				if (viz_global_plan)
					displayTrajectory(Minibot::Planner::global_plan, 0);
				else
					undisplayTrajectory(Minibot::Planner::global_plan, global_trajectory_name);
			}
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		}
		case minibot::PlanningAction::Request::VIS_LOCAL_PLAN: {
			viz_local_plan = req.jfdi;
			if (viz_global_plan)
				displayTrajectory(Minibot::Planner::global_plan, 0);
			else
				undisplayTrajectory(Minibot::Planner::global_plan, local_trajectory_name);

			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		}
		case minibot::PlanningAction::Request::STEP_FORWARD:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;

		default:
			ROS_ERROR_STREAM_NAMED (LOG_NAME, "Minibot::Database::handleDatabaseAction invalid type=" << req.type);
			pub_msg.publish(Minibot::Utils::createMsg(planner_prefix + err_msg_prefix  + "Don't know the database action " + std::to_string(req.type) ));
			res.error_code.val = minibot::ErrorCodes::FAILURE;
			return true;
		}
}

}

}
