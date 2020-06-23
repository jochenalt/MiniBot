#include "planner.h"
#include "database.h"
#include "node.h"
#include "utils.h"
#include "marker.h"
#include "execution.h"

#include <minibot/Programme.h>
#include <minibot/ErrorCodes.h>
#include <minibot/GlobalPlan.h>
#include <minibot/LocalPlan.h>
#include <minibot/Statement.h>

#include <trap_velocity_profile.h>

namespace Minibot {

minibot::GlobalPlan global_plan;

std::string planner_prefix = "PLANNER:";
std::string global_trajectory_name = "global_trajectory";
std::string local_trajectory_name= "local_trajectory";


namespace Planner {

int local_plan_index = -1;		// index of the programme.statement that is to be displayed as local plan
#define LOG_NAME "planner"


void construct() {
	ROS_INFO_STREAM_NAMED(LOG_NAME, "module planner init");
}

void init() {
	plan();
}

void  createGlobalPlan (const minibot::Configuration& settings,  minibot::Programme & prog, const minibot::PoseStorage& poses) {
	minibot::GlobalPlan tmp_global_plan;
	tmp_global_plan.error_code.val = minibot::ErrorCodes::SUCCESS; // think positive

	bool is_in_trajectory = false;
	int start_index = -1;
	int goal_index = -1;
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
					goal_index = idx;
				}
			}
			else {
				if (stmt.type == minibot::Statement::STATEMENT_TYPE_MOVEMENT) {
					number_of_points ++;
					goal_index = idx;
				} else {
					// a trajectory ends at a wait statement
					if (stmt.type == minibot::Statement::STATEMENT_TYPE_WAIT) {
						goal_index = idx;
					}
				}
			}
		} else {
			// outside a trajectory
			if (stmt.type == minibot::Statement::STATEMENT_TYPE_MOVEMENT) {
				start_index = idx;
				is_in_trajectory = true;
				number_of_points = 1;
			}
			else {
				if (stmt.type == minibot::Statement::STATEMENT_TYPE_WAYPOINT) {
					stmt.error_code.val = minibot::ErrorCodes::TRAJECTORY_CANNOT_START_WITH_WAYPOINT;
					tmp_global_plan.error_code.val = minibot::ErrorCodes::FAILURE;
				}
			}
		}

		// did we find a local plan?
		if ((start_index >= 0) && (goal_index >=0) && (number_of_points > 1)) {
			minibot::LocalPlan local_plan = createLocalPlan(prog, poses, start_index, goal_index);
			tmp_global_plan.local_plan.push_back(local_plan);
			start_index = goal_index;
			is_in_trajectory = true;
			goal_index = -1;
			number_of_points = 1;
			if (local_plan.error_code.val != minibot::ErrorCodes::SUCCESS)
				tmp_global_plan.error_code.val = minibot::ErrorCodes::FAILURE;
		}
	}
	Minibot::global_plan = tmp_global_plan;
}



minibot::LocalPlan createLocalPlan (minibot::Programme & prog, const minibot::PoseStorage& poses, int start_index, int goal_index) {
	minibot::LocalPlan local_plan;
	local_plan.start_index = start_index;
	local_plan.goal_index = goal_index;
    trajectory_msgs::JointTrajectory joint_trajectory;

    // gather start, end and waypoints in one convinient vector
    std::vector<minibot::MinibotState> waypoints;
    for (size_t idx = start_index;idx <= goal_index;idx++) {
    	minibot::Statement stmt = prog.statements[idx];
    	int pose_idx = Utils::findPose(poses, stmt.pose_uid);
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
				Minibot::Planner::planPTP(waypoints[idx], waypoints[idx+1], local_local_traj, 1.0,1.0);
				if (idx == 0)
					joint_trajectory = local_local_traj;
				else
					Utils::append(joint_trajectory, local_local_traj);
		    }
		    local_plan.joint_trajectory = joint_trajectory;
		    local_plan.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		}
		default:
			break;
	}
	return local_plan;
}

void undisplayTrajectory(const minibot::GlobalPlan& global_plan, std::string trajectory_name) {

	int last_index = 0;
	if (Minibot::global_plan.local_plan.size()> 0)
		last_index = Minibot::global_plan.local_plan.back().start_index;

	// undisplay any old trajectories beyond the last one
	for (int traj_idx = 0; traj_idx <= last_index; traj_idx++)
		Minibot::Gearwheel::deleteTrajectoryMarker(trajectory_name, traj_idx);
}

void displayTrajectory(const minibot::Configuration& settings, const minibot::GlobalPlan& global_plan, int preveous_last_index) {
	int old_traj_idx = 0; // index position from which old trajectories are deleted
	int color_no = 0;
	for (minibot::LocalPlan local_plan : global_plan.local_plan) {
		// undisplay old trajectories including the one that is about to be displayed (but maybe not)
		for (int old_traj_dx = old_traj_idx; old_traj_dx <local_plan.start_index; old_traj_dx++)
			Minibot::Gearwheel::deleteTrajectoryMarker(global_trajectory_name, old_traj_dx);
		old_traj_idx =local_plan.start_index;

		if (settings.vis_global_plan) {
			Minibot::Gearwheel::createTrajectoryMarker(local_plan.joint_trajectory, true, global_trajectory_name, local_plan.start_index, color_no);
			old_traj_idx = local_plan.start_index + 1; // don't delete this trajectory in the next loop
		}
		else {
			if (settings.vis_local_plan && local_plan.start_index <= local_plan_index && local_plan.goal_index >= local_plan_index ) {
				Minibot::Gearwheel::createTrajectoryMarker(local_plan.joint_trajectory, false, local_trajectory_name, local_plan.start_index, color_no);
				old_traj_idx = local_plan.start_index + 1;// don't delete this trajectory in the next loop
			}
		}
	}

	// undisplay any old trajectories beyond the last one
	for (int traj_dx = old_traj_idx; traj_dx <=preveous_last_index; traj_dx++)
		Minibot::Gearwheel::deleteTrajectoryMarker(global_trajectory_name, traj_dx);
}


void plan() {
	// find out the last index of the previously display trajectory
	int last_index = -1;
	if (Minibot::global_plan.local_plan.size()> 0)
		last_index = Minibot::global_plan.local_plan.back().start_index;

	createGlobalPlan(Minibot::settings, Minibot::programme_store, Minibot::pose_store);
	displayTrajectory(Minibot::settings, Minibot::global_plan , last_index);
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
		case minibot::PlanningAction::Request::SIMULATE_LOCAL_PLAN: {
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			for (size_t idx = 0; idx < global_plan.local_plan.size();idx++) {

				if ((req.start_index >= global_plan.local_plan[idx].start_index) &&
					(req.start_index < global_plan.local_plan[idx].goal_index)) {
					res.error_code = Minibot::Execution::execute(global_plan.local_plan[idx]);
				}
			}
			break;
		}
		case minibot::PlanningAction::Request::VIS_GLOBAL_PLAN: {
			settings.vis_global_plan = req.jfdi;
			if (settings.vis_global_plan)
				displayTrajectory(settings, Minibot::global_plan, 0);
			else
				undisplayTrajectory(Minibot::global_plan, global_trajectory_name);
			Minibot::Database::setSettings(settings);
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		}
		case minibot::PlanningAction::Request::VIS_LOCAL_PLAN: {
			Minibot::Database::readSettings();
			settings.vis_local_plan = req.jfdi;
			if (settings.vis_global_plan)
				displayTrajectory(settings, Minibot::global_plan, 0);
			else
				undisplayTrajectory(Minibot::global_plan, local_trajectory_name);
			Minibot::Database::setSettings(settings);

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
	return true;
}

}

}
