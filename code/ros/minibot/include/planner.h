#ifndef MININBOT_PLANNER_H_
#define MININBOT_PLANNER_H_

#include <map>

#include "ros/ros.h"
#include <minibot/PlanningAction.h>


namespace Minibot {


namespace Planner{
  void init();

  // handle incoming planning requests
  bool handlePlanningAction(minibot::PlanningAction::Request &req,
						  	minibot::PlanningAction::Response &res);
}

}

#endif
