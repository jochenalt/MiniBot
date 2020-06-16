/*
 * database.h
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#ifndef INCLUDE_DATABASE_H_
#define INCLUDE_DATABASE_H_

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <minibot/JointStateConfiguration.h>
#include <minibot/MinibotState.h>
#include <minibot/Configuration.h>

namespace Minibot {
namespace Database {

extern minibot::Configuration settings;

	// call me before anything happens in the database
void init(const ros::NodeHandle& nh);


}
}


#endif /* INCLUDE_DATABASE_H_ */
