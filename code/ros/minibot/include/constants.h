/*
 * constats.h
 *
 *  Created on: Jun 16, 2020
 *      Author: jochenalt
 */

#ifndef INCLUDE_CONSTANTS_H_
#define INCLUDE_CONSTANTS_H_

namespace Minibot {

const int joint_state_publish_rate = 25;	  // [Hz] frequency of topic joint_states
const double trajectory_sampling_time = 0.04; // [s] sample time to be used for trajectory generation (25Hz)

const double max_trans_vel= 1;
const double max_trans_acc =  2.25;
const double max_trans_dec = -5;
const double max_rot_vel = 1.57;

}

#endif /* INCLUDE_CONSTANTS_H_ */
