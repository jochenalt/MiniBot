#ifndef MININBOT_PLANNER_H_
#define MININBOT_PLANNER_H_

#include "ros/ros.h"
#include <minibot/PlanningAction.h>
#include <minibot/Programme.h>
#include <minibot/LocalPlan.h>
#include <minibot/GlobalPlan.h>


namespace Minibot {

namespace Planner{
  void init();

  // handle incoming planning requests
  bool handlePlanningAction(minibot::PlanningAction::Request &req,
						  	minibot::PlanningAction::Response &res);


  // compile a programme and create a global plan consisting of local plans
  minibot::GlobalPlan createGlobalPlan (minibot::Programme & prog);

  // compile a local plan with from start_index to end_index
  minibot::LocalPlan createLocalPlan (minibot::Programme & prog, int start_index, int end_index);

  void displayTrajectory(const minibot::GlobalPlan global_plan, const minibot::Programme & prog);
}

}

#endif
