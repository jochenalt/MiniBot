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

  /*
  std::vector<std::string> getJointNamesOfGroup(std::string group_name) {
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* jmg = kinematic_state->getJointModelGroup(group_name);
    if (jmg == NULL) {
        ROS_ERROR_STREAM("getJointNamesOfGroup: did not find group " << group_name);
        return NULL;
    }
    std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
    int pos_no = 0;
    for (size_t i = 0;i<joint_names.size(); i++) {
        const moveit::core::JointModel* jm = jmg->getJointModel(joint_names[i]);
        if (jm->getType() == moveit::core::JointModel::REVOLUTE)
  	kinematic_state->setVariablePosition(joint_names[i], joint_state.position[pos_no++]);
    }
  }
  */

}
}
