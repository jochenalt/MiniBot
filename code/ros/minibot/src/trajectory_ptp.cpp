/*
 * trajectory_ptp.cpp
 *
 *  Created on: Jun 20, 2020
 *      Author: jochen
 */



#include "constants.h"
#include "globals.h"
#include "kinematics.h"
#include "planner.h"
#include "utils.h"

#include "trap_velocity_profile.h"
#include "eigen_conversions/eigen_msg.h"
#include "moveit/robot_state/conversions.h"

#include <minibot/MinibotPose.h>
#include <minibot/MinibotState.h>

#include <uniform_sample_filter.h>
#include <iostream>
#include <sstream>

namespace Minibot {
namespace Planner {

#define LOG_NAME "planner"

bool planPTP(
		const minibot::MinibotState& start_pos,
		const minibot::MinibotState& goal_pos,
        trajectory_msgs::JointTrajectory &result_joint_trajectory,
        double velocity_scaling_factor,
        double acceleration_scaling_factor)
{
	bool ok = true;
    trajectory_msgs::JointTrajectory joint_trajectory;
	joint_trajectory.joint_names = Minibot::minibot_joint_names;

	// find the slowest joint that takes the longest for a normalized profile
	std::string leading_axis = Minibot::minibot_joint_names.front();
	double max_duration = -1.0; // negative, since we are looking for max
	std::map<std::string, TrapVelocityProfile> velocity_profile;
	for(const auto& joint_name : Minibot::minibot_joint_names)
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

	// synchronize all non-leading joints such that duration of all joints is the same
	double acc_time = velocity_profile.at(leading_axis).FirstPhaseDuration();
	double const_time = velocity_profile.at(leading_axis).SecondPhaseDuration();
	double dec_time = velocity_profile.at(leading_axis).ThirdPhaseDuration();

	for(const auto& joint_name : Minibot::minibot_joint_names) {
		if(joint_name != leading_axis)
	    {
	      // make full synchronization
	      // causes the program to terminate if acc_time<=0 or dec_time<=0 (should be prevented by goal_reached block above)
	      // by using the most strict limit, the following should always return true
	      if (!velocity_profile.at(joint_name).setProfileAllDurations(
	    		  Utils::getJointValue(start_pos.joint_state, joint_name),
	    		  Utils::getJointValue(goal_pos.joint_state, joint_name), acc_time,const_time,dec_time))
	      {
	        std::stringstream error_str;
	        error_str << "TrajectoryGeneratorPTP::planPTP(): Can not synchronize velocity profile of axis " << joint_name
	                  << " with leading axis " << leading_axis;
			ok = false;
			ROS_ERROR_STREAM_NAMED (LOG_NAME, "Minibot::Planner::planPTP could not smooth trajectory " );
			pub_msg.publish(Minibot::Utils::createMsg(planner_prefix + err_msg_prefix  + "cannot synchronize trajectory"));
	      }
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

	UniformSampleFilter smoothness_filter;
	smoothness_filter.configure(Minibot::trajectory_sampling_time);
	smoothness_filter.update(joint_trajectory, result_joint_trajectory);
}

}
}
