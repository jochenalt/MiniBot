#include "utils.h"

namespace Minibot {

#define LOG_NAME "utils"

std::map<int, std::string> return_code_map;
std::string err_msg_prefix = "ERR:";
std::string warn_msg_prefix = "WARN:";
std::string info_msg_prefix ="INFO:";
std::string kinematics_prefix = "KINEMATICS:";


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

  std_msgs::String createMsg(std::string msg) {
    std_msgs::String ros_msg;
    ros_msg.data = msg;
    return ros_msg;
  }

  // return the index of the joint with the passed name
  int findJoint(const sensor_msgs::JointState& joint_state, const std::string& s) {
	  for (size_t i = 0;i<joint_state.name.size();i++) {
		  if (joint_state.name[i] == s)
			  return i;
	  }
	  return -1;
  }

}
}
