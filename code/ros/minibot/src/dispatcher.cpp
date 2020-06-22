/*
 * dispatcher.cpp
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */


#include "dispatcher.h"
#include "kinematics.h"
#include "marker.h"

#include "node.h"

namespace Minibot {
namespace Dispatcher {

#define LOG_NAME "dispatcher"


// receive TCP input from UI and act accordingly:
// - compute IK
// - post new joint values and all configurations
void updateTCPCallback(const minibot::MinibotState& state) {

  // compute flange out of tcp pose, in case tool_length is > 0
  const geometry_msgs::Pose& flange_pose = Minibot::Kinematics::computeTCPBase(state.pose.pose, state.pose.tool_length);

  // compute IK relative to flange
  minibot::JointStateConfiguration jointStateConfiguration;
  bool foundIK = Minibot::Kinematics::computeIK(flange_pose, state.joint_state, jointStateConfiguration);

  if (foundIK > 0) {
	  for (size_t i = 0;i<jointStateConfiguration.configuration.size();i++)
		  Minibot::Kinematics::setEndEffectorPosition(jointStateConfiguration.configuration[i],
				  	  	  	  	  	  	  	  	  	  state.joint_state);

	  minibot::MinibotState new_state = state;
	  new_state.configuration = jointStateConfiguration.configuration;
	  new_state.joint_state = jointStateConfiguration.configuration[0];
	  new_state.pose.pose = state.pose.pose;
	  new_state.pose.tool_length = state.pose.tool_length;

	  // publish joint values and all configurations
	  pub_joint_state_ui.publish(new_state.joint_state);
	  pub_joint_values_config.publish(jointStateConfiguration);

	  // change the gearhweel
	  Minibot::Gearwheel::updateGearwheelPose(state.pose.pose);

	  // store this state
	  Minibot::Kinematics::setLastMinibotState(new_state);

  } else {
	  pub_msg.publish(Minibot::Utils::createMsg(kinematics_prefix + err_msg_prefix  + "Could not find inverse kinematic"));
  }

}

void updateExecutionJointStatesCallback(const sensor_msgs::JointState& joint_state) {
	ROS_ASSERT_MSG( joint_state.name.size() > 0, "updateExecutionJointStatesCallback called without joints");
	minibot::MinibotState minibot_state;
	minibot_state.joint_state = joint_state;
	updateJointStatesCallback(minibot_state);
}

// receive joint state input from UI and act accordingly:
// - compute FK
// - post new tcp values and all configurations
void updateJointStatesCallback(const minibot::MinibotState& state) {
	ROS_ASSERT_MSG( state.joint_state.name.size() > 0, "updateJointStatesCallback called without joints");

	geometry_msgs::Pose flange_pose;
	Minibot::Kinematics::computeFK(state.joint_state, flange_pose);

	geometry_msgs::Pose tcp_pose = Minibot::Kinematics::computeTCPTip(flange_pose, state.pose.tool_length);

	minibot::JointStateConfiguration jointStateConfguration;
	bool foundIK = Minibot::Kinematics::computeIK(tcp_pose, state.joint_state, jointStateConfguration);
	if (foundIK > 0) {
		  for (size_t i = 0;i<jointStateConfguration.configuration.size();i++)
			  Minibot::Kinematics::setEndEffectorPosition(jointStateConfguration.configuration[i],
					  	  	  	  	  	  	  	  	  	  state.joint_state);

		  // bu
		  minibot::MinibotState new_state = state;
		  new_state.pose.pose = tcp_pose;
		  new_state.configuration = jointStateConfguration.configuration;
		  new_state.joint_state = state.joint_state;
		  new_state.pose.tool_length = state.pose.tool_length;


		  // publish new tcp to UI
		  pub_tcp_ui.publish(new_state.pose);

		  // update the position of gearhweel
		  Minibot::Gearwheel::updateGearwheelPose(new_state.pose.pose);

		  // publish tcp and all configuration
		  pub_joint_state_ui.publish(new_state.joint_state);
		  pub_joint_values_config.publish(jointStateConfguration);

		  Minibot::Kinematics::setLastMinibotState(new_state);
	  } else {
		  pub_msg.publish(Minibot::Utils::createMsg(kinematics_prefix + err_msg_prefix  + "Could not find inverse kinematic"));
	  }
}

// receive a configuration of the existing tcp
void updateJointStatesConfigurationCallback(const minibot::MinibotState& state) {

	if (state.joint_state.name.size() == 0) {
		ROS_ERROR_STREAM_NAMED(LOG_NAME,"updateJointStatesConfigurationCallback call without joints");
		return;
	}

	// publish tcp and all configuration
	pub_joint_state_ui.publish(state.joint_state);
    Minibot::Kinematics::setLastMinibotState(state);
}


// receive TCP input from UI and act accordingly:
// - compute IK
// - post new joint values and all configurations
void updateGearwheelCallback(const geometry_msgs::Pose& pose) {

   minibot::MinibotState new_state;

  // compute flange out of tcp pose, in case tool_distance is > 0
  minibot::MinibotState state = Minibot::Kinematics::getLastMinibotState();
  const geometry_msgs::Pose& flange_pose = Minibot::Kinematics::computeTCPBase(pose, state.pose.tool_length);

  // compute IK relative to flange
  minibot::JointStateConfiguration jointStateConfguration;
  bool foundIK = Minibot::Kinematics::computeIK(flange_pose, state.joint_state, jointStateConfguration);

  if (foundIK > 0) {
	  for (size_t i = 0;i<jointStateConfguration.configuration.size();i++)
		  Minibot::Kinematics::setEndEffectorPosition(jointStateConfguration.configuration[i],
				  	  	  	  	  	  	  	  	  	  state.joint_state);
	  new_state.configuration = jointStateConfguration.configuration;
	  new_state.pose.pose = pose;
	  new_state.joint_state = jointStateConfguration.configuration[0];
	  new_state.pose.tool_length = state.pose.tool_length;


	  // publish new tcp to UI
	  pub_tcp_ui.publish(new_state.pose);

	  // publish joint values and all configurations
	  pub_joint_state_ui.publish(new_state.joint_state);
	  pub_joint_values_config.publish(jointStateConfguration);

	  // save the state for later use
	  Minibot::Kinematics::setLastMinibotState(new_state);

  } else {
	  pub_msg.publish(Minibot::Utils::createMsg(kinematics_prefix + err_msg_prefix  + "Could not find inverse kinematic"));
  }

}

}
}
