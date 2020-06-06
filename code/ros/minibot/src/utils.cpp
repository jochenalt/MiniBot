#include "utils.h"

namespace Minibot {

#define LOG_NAME "utils"

std::map<int, std::string> return_code_map;


namespace Utils {

  // robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  robot_model_loader::RobotModelLoader* robot_model_loader_ptr = NULL;

  void init() {

  return_code_map = {
	{ ReturnCode::OK, "OK" },
	{ ReturnCode::IK_NOT_AVAILABLE, "One or more ik solutions could not be found" },
	{ ReturnCode::JOINT_OUT_OF_POSITION_BOUNDS, "one or more joints are out of position bounds" },
	{ ReturnCode::UKNOWN, "unknown error" }
  };
  robot_model_loader_ptr = new robot_model_loader::RobotModelLoader(robot_description);

  }

  robot_model::RobotModelPtr getRobotModel() {
    kinematic_model = robot_model_loader_ptr->getModel();
    return kinematic_model;
  }

}
}
