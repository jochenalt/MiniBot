/*
 * node.h
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#ifndef INCLUDE_NODE_H_
#define INCLUDE_NODE_H_

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>


// publisher of new joint values
extern ros::Publisher pub_joint_state_ui;

// publisher of possible configurations
extern ros::Publisher pub_joint_values_config;

// publish messages to UI
extern ros::Publisher pub_msg;

// publish new tcp to UI
extern ros::Publisher pub_tcp_ui;

// publish updates from gearwheel
extern ros::Publisher pub_gearwheel_pose;

// publisher for messages
extern ros::Publisher pub_msg;

// publisher for global trajectories
extern ros::Publisher pub_global_plan;

// publisher for local trajectories
extern ros::Publisher pub_local_plan;



#endif /* INCLUDE_NODE_H_ */
