#include "planner.h"
#include "database.h"
#include "node.h"
#include "utils.h"

#include <minibot/Programme.h>
#include <minibot/ErrorCodes.h>
#include <minibot/GlobalPlan.h>
#include <minibot/LocalPlan.h>

namespace Minibot {

#define LOG_NAME "planner"

std::string planner_prefix = "PLANNER:";

namespace Planner {

bool viz_global_plan = false;
bool viz_local_plan = false;

void init() {
	ROS_INFO_STREAM_NAMED(LOG_NAME, "module planner init");
}


minibot::GlobalPlan createGlobalPlan (minibot::Programme & prog) {
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
			minibot::LocalPlan local_plan = createLocalPlan(prog, start_index, end_index);
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


minibot::LocalPlan createLocalPlan (minibot::Programme & prog, int start_index, int end_index) {
	minibot::LocalPlan local_plan;
	local_plan.start_index = start_index;
	local_plan.end_index = end_index;

	return local_plan;
}

void displayTrajectory(const minibot::GlobalPlan global_plan, const minibot::Programme & prog) {

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
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::CLEAR_PLAN:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::GLOBAL_PLAN: {
			minibot::Programme prgm = Minibot::Database::getProgramme();
			minibot::GlobalPlan global_plan = createGlobalPlan(prgm);
			displayTrajectory(global_plan, prgm);
			res.error_code.val = global_plan.error_code.val;
			break;
		}
		case minibot::PlanningAction::Request::VIS_GLOBAL_PLAN: {
			viz_global_plan = req.jfdi;
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		}
		case minibot::PlanningAction::Request::VIS_LOCAL_PLAN: {
			viz_local_plan = req.jfdi;
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
