/*
 * node.h
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#ifndef INCLUDE_NODE_H_
#define INCLUDE_NODE_H_


// publisher of new joint values
extern ros::Publisher pub_joint_state_ui;

// publisher of possible configurations
extern ros::Publisher pub_joint_values_config;

// publish messages to UI
extern ros::Publisher pub_msg;

// publish new tcp to UI
extern ros::Publisher pub_tcp_ui;




#endif /* INCLUDE_NODE_H_ */
