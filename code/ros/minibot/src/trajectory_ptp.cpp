/*
 * trajectory_ptp.cpp
 *
 *  Created on: Jun 20, 2020
 *      Author: jochen
 */



#include "utils.h"
#include "constants.h"
#include "kinematics.h"

#include "trap_velocity_profile.h"
#include "ros/ros.h"
#include "eigen_conversions/eigen_msg.h"
#include "moveit/robot_state/conversions.h"

#include <minibot/MinibotPose.h>
#include <minibot/MinibotState.h>


#include <iostream>
#include <sstream>

namespace Minibot {
namespace Planner {


void planPTP(
		const minibot::MinibotState& start_pos,
		const minibot::MinibotState& goal_pos,
        trajectory_msgs::JointTrajectory &joint_trajectory,
        double velocity_scaling_factor,
        double acceleration_scaling_factor)
{

	joint_trajectory.joint_names = Minibot::Kinematics::minibot_joint_names;

	// find the slowest joint that takes the longest for a normalized profile
	std::string leading_axis = Minibot::Kinematics::minibot_joint_names.front();
	double max_duration = -1.0; // negative, since we are looking for max
	std::map<std::string, TrapVelocityProfile> velocity_profile;
	for(const auto& joint_name : Minibot::Kinematics::minibot_joint_names)
	{
		// create velocity profile if necessary
		velocity_profile.insert(std::make_pair(
								  joint_name,
								  TrapVelocityProfile(
									velocity_scaling_factor * Utils::getJointLimits(joint_name).max_velocity,
									acceleration_scaling_factor * Utils::getJointLimits(joint_name).max_acceleration,
									acceleration_scaling_factor * Utils::getJointLimits(joint_name).max_acceleration)));

		velocity_profile.at(joint_name).SetProfile(
											Utils::getJointValue(start_pos.joint_state, joint_name),
											Utils::getJointValue(goal_pos.joint_state,joint_name));

		// if moving this joint takes longer than the slowest joint, switch the leading axis
		if(velocity_profile.at(joint_name).Duration() > max_duration)
		{
		  max_duration = velocity_profile.at(joint_name).Duration();
		  leading_axis = joint_name;
		}
	}

	// construct joint trajectory point
	// squeeze the sampling time such that it fits exactly
	int samples = std::round(max_duration/trajectory_sampling_time);
	for(int sample = 0;sample < samples;sample++) {
		double t_sample=(max_duration*sample)/(samples-1);
		trajectory_msgs::JointTrajectoryPoint point;
		point.time_from_start =  ros::Duration(t_sample);
		for(std::string & joint_name : joint_trajectory.joint_names)
		{
		  point.positions.push_back(velocity_profile.at(joint_name).Pos(t_sample));
		  point.velocities.push_back(velocity_profile.at(joint_name).Vel(t_sample));
		  point.accelerations.push_back(velocity_profile.at(joint_name).Acc(t_sample));
		}
		joint_trajectory.points.push_back(point);
	}
}

}
}
