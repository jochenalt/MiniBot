/*
 * dispatcher.h
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#ifndef INCLUDE_DISPATCHER_H_
#define INCLUDE_DISPATCHER_H_

#include <geometry_msgs/Pose.h>
#include <minibot/JointStateConfiguration.h>
#include <minibot/MinibotState.h>

namespace Minibot {
namespace Dispatcher {

	  // callback for tcp input from UI
	void updateTCPCallback(const minibot::MinibotState&);

	  // callback for joint_input from UI
	void updateJointStatesCallback(const minibot::MinibotState&);

		// callback for joint input from execution of a trajectory
	void updateExecutionJointStatesCallback(const sensor_msgs::JointState& joint_state);


		// callback for a different configuration of the same tcp
	void updateJointStatesConfigurationCallback(const minibot::MinibotState& state);

		// callback of gearwheel
	void updateGearwheelCallback(const geometry_msgs::Pose& pose);

}
}


#endif /* INCLUDE_DISPATCHER_H_ */
