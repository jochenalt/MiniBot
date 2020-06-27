/*
 * trajectory_lin.cpp
 *
 *  Created on: Jun 24, 2020
 *      Author: jochen
 */




#include <ros/ros.h>
#include <time.h>
#include <cassert>
#include <sstream>
#include "planner.h"
#include "kinematics.h"
#include "constants.h"

#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include "trap_velocity_profile.h"
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <uniform_sample_filter.h>

#include <splines.hpp>
#include <Vec3.hpp>

namespace Minibot {
namespace Planner {

#define LOG_NAME "dispatcher"


void logIsometry3d(const Eigen::Isometry3d& p) {
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(p, pose);
    ROS_INFO_STREAM_NAMED(LOG_NAME, "Point(pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z << ")"
				   << " ori=(x=" << pose.orientation.x << " y=" << pose.orientation.y << " z=" << pose.orientation.z << " w=" << pose.orientation.w <<") ");
}

double cartesianDistance(const std::vector<minibot::MinibotState>& waypoints) {
	double distance = 0;
	for (std::size_t i = 0; i < waypoints.size(); ++i) {
		Eigen::Isometry3d end, start;
		tf::poseMsgToEigen (waypoints[i].pose.pose, start);
		tf::poseMsgToEigen (waypoints[i+1].pose.pose, end);
		distance += (start.translation() - end.translation()).norm();
	}
	return distance;
}

Eigen::Isometry3d getPathPoint( double ratio, const Eigen::Isometry3d& start, const Eigen::Isometry3d& end) {

	const Eigen::Quaterniond start_quaternion(start.rotation());
	const Eigen::Quaterniond end_quaternion(end.rotation());
	const Eigen::Vector3d start_translation = start.translation();
	const Eigen::Vector3d end_translation = end.translation();
	Eigen::Isometry3d eigen_pose;
	eigen_pose.translation() = (end_translation - start_translation) * ratio + start_translation;
	eigen_pose.linear() = start_quaternion.slerp(ratio, end_quaternion).toRotationMatrix();
	return eigen_pose;
}

Eigen::Isometry3d getIsometry(const minibot::MinibotState& state) {
	Eigen::Isometry3d iso;
	tf::poseMsgToEigen (state.pose.pose, iso);
	return iso;
}

bool getPathPoint( double ratio, const minibot::MinibotState& start, const minibot::MinibotState& goal, minibot::MinibotState& result) {

	Eigen::Isometry3d start_iso = getIsometry(start);
	Eigen::Isometry3d goal_iso = getIsometry(goal);
	Eigen::Isometry3d result_iso = getPathPoint(ratio, start_iso, goal_iso);

	tf::poseEigenToMsg(result_iso, result.pose.pose);
	minibot::JointStateConfiguration solutions;

	bool ik_ok = Minibot::Kinematics::computeIK(result.pose.pose, start.joint_state,solutions);
	if (ik_ok) {
		// add the gripper, take care that the joints are listed in the same order like in goal
		// (later on, all this is concatenated to a trajectory when it needs to be in the same order)
		for (size_t i = 0;i<solutions.configuration.size();i++) {
			solutions.configuration[i].name.resize(minibot_joint_names.size());
			solutions.configuration[i].position.resize(minibot_joint_names.size());
			for (size_t j = 0;j<minibot_gripper_joint_names.size();j++) {
				std::string joint_name = minibot_gripper_joint_names[j];
				double start_pos = Utils::getJointValue(start.joint_state, joint_name);
				double goal_pos = Utils::getJointValue(goal.joint_state, joint_name);
				int idx = Utils::findJoint(goal.joint_state, joint_name);
				solutions.configuration[i].name[idx] = joint_name;
				solutions.configuration[i].position[idx]= start_pos + ratio * (goal_pos-start_pos);
			}
		}

		result.configuration = solutions.configuration;
		result.joint_state = solutions.configuration[0];
	} else {
		return false;
	}
	return ik_ok;
}

double getMaxVelocityDuration(const minibot::MinibotState& start, const minibot::MinibotState& goal) {

	double max_duration= 0;
	for(int joint_idx = 0;joint_idx < Minibot::minibot_joint_names.size();joint_idx++)
   	{
		std::string joint_name = minibot_joint_names[joint_idx];
		double max_vel = Utils::getJointLimits(joint_name).max_velocity;
		double start_pos = Utils::getJointValue(start.joint_state, joint_name);
		ROS_ASSERT(start_pos != -1);
		double goal_pos = Utils::getJointValue(goal.joint_state, joint_name);
		ROS_ASSERT(goal_pos != -1);

		double time = std::abs(start_pos-goal_pos)/max_vel;
		if (time > max_duration) {
			max_duration = time;
		}

   	}
	return max_duration;
}

bool generateTrajectoryRecursiv(const minibot::MinibotState& start, const minibot::MinibotState& goal,
	    				std::vector<minibot::MinibotState>& trajectory, double left, double right) {
    ROS_INFO_STREAM_NAMED(LOG_NAME, "call " << (left+right)/2 << " [" << left << " " << right<< "]");

    trajectory.clear();
	minibot::MinibotState middle;
	bool ok = getPathPoint(0.5, start, goal, middle);
	double left_duration = getMaxVelocityDuration(start, middle);
	double right_duration = getMaxVelocityDuration(middle, goal);
	bool inserted = false;
	std::vector<minibot::MinibotState> left_trajectory;
	std::vector<minibot::MinibotState> right_trajectory;

	if (ok && (left_duration > trajectory_sampling_time)) {
		ok = generateTrajectoryRecursiv(start,middle, left_trajectory, left, (left+right)/2);
	}

	if (ok && (right_duration > trajectory_sampling_time)) {
		ok = generateTrajectoryRecursiv(middle,goal, right_trajectory, (left+right)/2, right);
	}

	trajectory.insert( trajectory.begin(), left_trajectory.begin(), left_trajectory.end() );
	trajectory.push_back(middle);
	trajectory.insert( trajectory.end(), right_trajectory.begin(), right_trajectory.end() );

	/*
	ROS_INFO_STREAM_NAMED(LOG_NAME,   "[" << std::setprecision(4) << left << " " << (left+right)/2 << " " << right<< "] " <<
							"pos=(x=" << middle.pose.pose.position.x << " y=" << middle.pose.pose.position.y << " z=" << middle.pose.pose.position.z);
	for (int i = 0;i<left_trajectory.size();i++) {
		geometry_msgs::Pose pose= left_trajectory[i].pose.pose;

		ROS_INFO_STREAM_NAMED(LOG_NAME, "   L" << std::setprecision(4) << i << " pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z);
	}
	ROS_INFO_STREAM_NAMED(LOG_NAME, "   M" << std::setprecision(4) << " pos=(x=" << middle.pose.pose.position.x << " y=" << middle.pose.pose.position.y << " z=" << middle.pose.pose.position.z);

	for (int i = 0;i<right_trajectory.size();i++) {
		geometry_msgs::Pose pose= right_trajectory[i].pose.pose;

		ROS_INFO_STREAM_NAMED(LOG_NAME, "   R" << std::setprecision(4) << i << " pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z);
	}

	for (int i = 0;i<trajectory.size();i++) {
		geometry_msgs::Pose pose= trajectory[i].pose.pose;

		ROS_INFO_STREAM_NAMED(LOG_NAME, "   R" << std::setprecision(4) << i << " pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z);
	}
	*/
	return ok;
}



bool generateTrajectory(const minibot::MinibotState& start, const minibot::MinibotState& goal,
	    				std::vector<minibot::MinibotState>& trajectory) {
	bool result = true;
	bool ok = generateTrajectoryRecursiv(start, goal, trajectory, 0,1);
	trajectory.insert(trajectory.begin(),start);

	return ok;
}

bool generateTrajectory(const std::vector<minibot::MinibotState>& waypoints,
	    				std::vector<minibot::MinibotState>& trajectory) {
	bool ok = true;
	for (int waypoint_idx = 0;waypoint_idx < waypoints.size()-1;waypoint_idx++) {
		std::vector<minibot::MinibotState> local_trajectory;
		ok = ok && generateTrajectory(waypoints[waypoint_idx], waypoints[waypoint_idx+1],local_trajectory);
		trajectory.insert(trajectory.end(), local_trajectory.begin(), local_trajectory.end());
	}
	trajectory.push_back(waypoints[waypoints.size()-1]);
	return ok;
}


bool planCartesianPath(const std::vector<minibot::MinibotState>& waypoints, trajectory_msgs::JointTrajectory& result_local_traj) {
	trajectory_msgs::JointTrajectory local_local_traj;
	std::vector<minibot::MinibotState> trajectory;
	for (int i = 0;i<waypoints.size();i++) {
		geometry_msgs::Pose pose= waypoints[i].pose.pose;
	    ROS_DEBUG_STREAM_NAMED(LOG_NAME, "Plan(pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z << ")"
					   << " ori=(x=" << pose.orientation.x << " y=" << pose.orientation.y << " z=" << pose.orientation.z << " w=" << pose.orientation.w <<") ");

	}

	bool ok = generateTrajectory(waypoints,trajectory);

	trajectory_msgs::JointTrajectory joint_trajectory;
	joint_trajectory.joint_names = trajectory[0].joint_state.name;
	for (int i = 0;i<trajectory.size();i++) {
		trajectory_msgs::JointTrajectoryPoint point;
		point.positions = trajectory[i].joint_state.position;
		joint_trajectory.points.push_back(point);
	}

	robot_trajectory::RobotTrajectory robot_trajectory(Utils::getRobotModel(), minibot_group_name);
	moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(Utils::getRobotModel()));
	kinematic_state->setToDefaultValues();
	for (size_t i = 0;i<Minibot::minibot_joint_names.size(); i++) {
	    kinematic_state->setVariablePosition(Minibot::minibot_joint_names[i], joint_trajectory.points[0].positions[i]);
	}
	robot_trajectory.setRobotTrajectoryMsg(*kinematic_state, joint_trajectory);
	trajectory_processing::IterativeParabolicTimeParameterization iptp;
	bool success = iptp.computeTimeStamps(robot_trajectory);

	moveit_msgs::RobotTrajectory robot_trajectory_msg;
	robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);

	local_local_traj.points = robot_trajectory_msg.joint_trajectory.points;
	local_local_traj.joint_names = robot_trajectory_msg.joint_trajectory.joint_names;

	for (int i = 0;i<trajectory.size();i++) {
		trajectory_msgs::JointTrajectoryPoint point;
		point.positions = trajectory[i].joint_state.position;
		joint_trajectory.points.push_back(point);
		geometry_msgs::Pose pose= trajectory[i].pose.pose;
		ROS_DEBUG_STREAM_NAMED(LOG_NAME, "Point(pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z << ")"
	    	    << "joint orig" << trajectory[i].joint_state
	    	    << "joint copy" << local_local_traj.points[i]

				<< " ori=(x=" << pose.orientation.x << " y=" << pose.orientation.y << " z=" << pose.orientation.z << " w=" << pose.orientation.w <<") ");

	}

	UniformSampleFilter smoothness_filter;
	smoothness_filter.configure(Minibot::trajectory_sampling_time);
	smoothness_filter.update(local_local_traj, result_local_traj);


	return ok;
};

}
}

