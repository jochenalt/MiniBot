#ifndef MINIBOT_UTILS_H_
#define MINIBOT_UTILS_H_


#include <map>

#include "ros/ros.h"
#include "ros/static_assert.h"

#include "std_msgs/String.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/JointLimits.h>

#include <minibot/PoseStorage.h>

extern std::string err_msg_prefix;
extern std::string warn_msg_prefix;
extern std::string info_msg_prefix;


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

namespace Utils {
  void construct();

  // @brief returns the name of the base link, probably "base_link"
  std::string getBaseFrameName();

  robot_model::RobotModelPtr getRobotModel();

  // create a message to be sent to the UI
  std_msgs::String createMsg(std::string msg);

  // return the index of the joint in joint_state with the name s
  int findJoint(const sensor_msgs::JointState& joint_state, const std::string& s);

  // return the join limits of the passed joint
  const moveit_msgs::JointLimits& getJointLimits(const std::string joint_name);

  // return the position of a certain joint
  double getJointValue(const sensor_msgs::JointState& joint_state, const std::string& s);

  // return a pose from the pose storage
  int findPose(const minibot::PoseStorage& poses, const int uid);


  // concatenate two trajectories and adapt their timing
  void append (trajectory_msgs::JointTrajectory& traj, const trajectory_msgs::JointTrajectory& add);

}


}

#endif
