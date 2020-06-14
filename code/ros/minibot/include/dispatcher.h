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

}
}


#endif /* INCLUDE_DISPATCHER_H_ */
