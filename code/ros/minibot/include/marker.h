/*
 * marker.h
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#ifndef INCLUDE_MARKER_H_
#define INCLUDE_MARKER_H_

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

namespace Minibot {
namespace Gearwheel {

	// call me before anything else, but after node initialisation
	void init();

	void updateGerwheelPose(const geometry_msgs::Pose& pose);
}
}


#endif /* INCLUDE_MARKER_H_ */
