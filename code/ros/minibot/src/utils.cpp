#include <map>
#include <vector>
#include "utils.h"

namespace Minibot {

#define LOG_NAME "utils"

std::string err_msg_prefix = "ERR:";
std::string warn_msg_prefix = "WARN:";
std::string info_msg_prefix ="INFO:";

std::string kinematics_prefix = "KINEMATICS:";
std::string posestore_prefix = "POSESTORE:";
std::string programme_prefix = "PROGRAMME:";
std::string settings_prefix = "SETTINGS:";

namespace Utils {

  // robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  robot_model_loader::RobotModelLoader* robot_model_loader_ptr = NULL;
  std::map<std::string, moveit_msgs::JointLimits> joint_limits;

  void init() {
	ROS_INFO_STREAM_NAMED(LOG_NAME, "module utils  init");
    robot_model_loader_ptr = new robot_model_loader::RobotModelLoader(robot_description);
    kinematic_model = robot_model_loader_ptr->getModel();

    // cache all joint limits
    // position limits are taken from minibot.urdf.xacro
    // velocity and acceleration limits are defined in joint_limits.yaml
    auto joint_models = kinematic_model->getActiveJointModels();
    for (auto joint_model : joint_models) {
    	const std::vector<moveit_msgs::JointLimits> joint_limit = joint_model->getVariableBoundsMsg();
    	ROS_ASSERT_MSG(joint_limit.size() == 1, "Utils::init missing or too many joint limits");
    	joint_limits.insert(std::make_pair(joint_model->getName(), joint_limit[0]));
    }
  }

  std::string getBaseFrameName() {
	  return "base_link";
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

  // return the index of the joint with the passed name
  double getJointValue(const sensor_msgs::JointState& joint_state, const std::string& s) {
	  for (size_t i = 0;i<joint_state.name.size();i++) {
		  if (joint_state.name[i] == s)
			  return joint_state.position[i];
	  }
	  return -1;
  }

  const moveit_msgs::JointLimits& getJointLimits(const std::string joint_name) {
	  return joint_limits[joint_name];
  }

  // return a pose idx from the pose storage, -1 if not found
  int findPose(const minibot::PoseStorage& poses, const int uid) {
	  for (size_t idx = 0;idx < poses.states.size();idx++) {
		  if (poses.states[idx].uid == uid)
			  return idx;
	  }
	  return -1;
  }

  bool isInArray(const std::string &value, const std::vector<std::string> &array) {
      return std::find(array.begin(), array.end(), value) != array.end();
  }

  void append (trajectory_msgs::JointTrajectory& traj, const trajectory_msgs::JointTrajectory& add) {
	  // check compatibility
	  ROS_ASSERT_MSG(traj.joint_names.size() == add.joint_names.size(), "utils::append trajectories joints not compatible");
	  for (int idx = 0;idx<traj.joint_names.size();idx++) {
		  ROS_ASSERT_MSG(isInArray(traj.joint_names[idx], add.joint_names),"Utils::append trajectories joint names not compatible");
	  }

	  // concatenate
	  ros::Duration time_from_start_offset = traj.points.back().time_from_start;
	  for (int idx = 0;idx < add.points.size();idx++) {
		  trajectory_msgs::JointTrajectoryPoint p = add.points[idx];
		  p.time_from_start += time_from_start_offset;
		  traj.points.push_back(p);
	  }
  }

}
}
