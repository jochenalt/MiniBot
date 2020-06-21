/*
 * trajectory_ptp.cpp
 *
 *  Created on: Jun 20, 2020
 *      Author: jochen
 */




#include "utils.h"
#include "constants.h"

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


	// find the slowest joint that takes the longest for a normalized profile
	std::string leading_axis = joint_trajectory.joint_names.front();
	double max_duration = -1.0; // negative, since we are looking for max
	std::map<std::string, TrapVelocityProfile> velocity_profile;
	for(const auto& joint_name : joint_trajectory.joint_names)
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
		if(velocity_profile.at(joint_name).Duration() > max_duration)
		{
		  max_duration = velocity_profile.at(joint_name).Duration();
		  leading_axis = joint_name;
		}
	}

	// generate the time samples upfront to garanty that the last time sample hits max_duration exactly
	std::vector<double> time_samples;
	for(double t_sample=0.0; t_sample<max_duration; t_sample += trajectory_sampling_time)
		time_samples.push_back(t_sample);
	// add last time
	time_samples.push_back(max_duration);

	// construct joint trajectory point
	for(double time_stamp : time_samples)
	{
		trajectory_msgs::JointTrajectoryPoint point;
		point.time_from_start =  ros::Duration(time_stamp);
		for(std::string & joint_name : joint_trajectory.joint_names)
		{
		  point.positions.push_back(velocity_profile.at(joint_name).Pos(time_stamp));
		  point.velocities.push_back(velocity_profile.at(joint_name).Vel(time_stamp));
		  point.accelerations.push_back(velocity_profile.at(joint_name).Acc(time_stamp));
		}
		joint_trajectory.points.push_back(point);
	}

	// Set last point velocity and acceleration to zero, just in case we have some rounding errors
	std::fill(joint_trajectory.points.back().velocities.begin(),
			  joint_trajectory.points.back().velocities.end(),
			  0.0);
	std::fill(joint_trajectory.points.back().accelerations.begin(),
			  joint_trajectory.points.back().accelerations.end(),
			  0.0);
}

}
}
