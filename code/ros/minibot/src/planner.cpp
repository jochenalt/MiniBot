#include "planner.h"
#include "node.h"
#include "utils.h"

namespace Minibot {

#define LOG_NAME "planner"

std::string planner_prefix = "PLANNER:";

namespace Planner {
void init() {
	ROS_INFO_STREAM_NAMED(LOG_NAME, "module planner init");
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
		case minibot::PlanningAction::Request::GLOBAL_PLAN:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::VIS_GLOBAL_PLAN:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
		case minibot::PlanningAction::Request::VIS_LOCAL_PLAN:
			res.error_code.val = minibot::ErrorCodes::SUCCESS;
			break;
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
