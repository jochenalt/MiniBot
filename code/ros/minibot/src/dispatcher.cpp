/*
 * dispatcher.cpp
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */


#include "dispatcher.h"
#include "kinematics.h"
#include "node.h"

namespace Minibot {
namespace Dispatcher {

#define LOG_NAME "dispatcher"

// receive TCP input from UI and act accordingly:
// - compute IK
// - post new joint values and all configurations
void updateTCPCallback(const minibot::MinibotState& state) {

  // compute flange out of tcp pose, in case tool_distance is > 0
  const geometry_msgs::Pose& flange_pose = Minibot::Kinematics::computeTCPBase(state.pose, state.tool_distance);

  // compute IK relative to flange
  minibot::JointStateConfiguration jointStateConfguration;
  bool foundIK = Minibot::Kinematics::computeIK(flange_pose, state.joint_state, jointStateConfguration);

  if (foundIK > 0) {
	  for (size_t i = 0;i<jointStateConfguration.configuration.size();i++)
		  Minibot::Kinematics::setEndEffectorPosition(jointStateConfguration.configuration[i],
				  	  	  	  	  	  	  	  	  	  state.joint_state);

	  // 0th configuration is closest to current joint state.
	  sensor_msgs::JointState joint_state = jointStateConfguration.configuration[0];

	  // publish joint values and all configurations
	  pub_joint_state_ui.publish(joint_state);
	  pub_joint_values_config.publish(jointStateConfguration);
  } else {
	  pub_msg.publish(Minibot::Utils::createMsg(kinematics_prefix + err_msg_prefix  + "Could not find inverse kinematic"));
  }

}

// receive joint state input from UI and act accordingly:
// - compute FK
// - post new tcp values and all configurations
void updateJointStatesCallback(const minibot::MinibotState& state) {

	if (state.joint_state.name.size() == 0) {
		ROS_ERROR_STREAM_NAMED(LOG_NAME,"updateJointStatesCallback call without joints");
		return;
	}
	geometry_msgs::Pose flange_pose;
	Minibot::Kinematics::computeFK(state.joint_state, flange_pose);

	geometry_msgs::Pose tcp_pose = Minibot::Kinematics::computeTCPTip(flange_pose, state.tool_distance);

	minibot::JointStateConfiguration jointStateConfguration;
	bool foundIK = Minibot::Kinematics::computeIK(tcp_pose, state.joint_state, jointStateConfguration);
	if (foundIK > 0) {
		  for (size_t i = 0;i<jointStateConfguration.configuration.size();i++)
			  Minibot::Kinematics::setEndEffectorPosition(jointStateConfguration.configuration[i],
					  	  	  	  	  	  	  	  	  	  state.joint_state);

		  minibot::MinibotPose pose;
		  pose.tool_distance = state.tool_distance;
		  pose.pose = flange_pose;

		  // pubish new tcp to UI
		  pub_tcp_ui.publish(pose);

		  // publish tcp and all configuration
		  sensor_msgs::JointState joint_state = jointStateConfguration.configuration[0];
		  pub_joint_state_ui.publish(state.joint_state);
		  pub_joint_values_config.publish(jointStateConfguration);
	  } else {
		  pub_msg.publish(Minibot::Utils::createMsg(kinematics_prefix + err_msg_prefix  + "Could not find inverse kinematic"));
	  }
}


}
}
