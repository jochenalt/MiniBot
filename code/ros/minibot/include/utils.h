#ifndef MINIBOT_UTILS_H_
#define MINIBOT_UTILS_H_

#include <map>

#include "ros/ros.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>


namespace Minibot {

enum ReturnCode
{
  OK = 1,
  IK_NOT_AVAILABLE = -1,
  JOINT_OUT_OF_POSITION_BOUNDS = -2,
  UKNOWN = -99
};


//  group names as defined in SRDF
static const std::string minibot_arm_group_name = "minibot_arm";
static const std::string minibot_gripper_group_name = "minibot_gripper";
static const std::string minibot_group_name = "minibot";

// param name with URDF
static const std::string robot_description= "robot_description";

extern std::map<int, std::string> return_code_map;

namespace Utils {
  void init();
  robot_model::RobotModelPtr getRobotModel();
}

}

#endif
