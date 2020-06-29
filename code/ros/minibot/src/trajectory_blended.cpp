/*
 * trajectory_lin.cpp
 *
 *  Created on: Jun 24, 2020
 *      Author: jochen
 */




#include <time.h>
#include <cassert>
#include <sstream>

#include "constants.h"
#include "globals.h"

#include "planner.h"
#include "kinematics.h"


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


extern void logIsometry3d(const Eigen::Isometry3d& p);
extern double cartesianDistance(const std::vector<minibot::MinibotState>& waypoints);
extern Eigen::Isometry3d getIsometry(const minibot::MinibotState& state);
extern double getMaxVelocityDuration(const minibot::MinibotState& start, const minibot::MinibotState& goal);


// compute the angle between a-b and b-c
double getTriangleAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
	Eigen::Vector3d ab = (a-b);
	Eigen::Vector3d cb = (c-b);
	double angle = std::atan2(ab.cross(cb).norm(), ab.dot(cb));
	return angle;
}


Eigen::Vector3d intersectVectorCircle(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double distance) {
	return a + Eigen::Vector3d(b-a).normalized()*distance;
}


// compute a point that is on the blended cartesian path, definded by waypoints
// blend_radius is reduced if two big for the waypoint (sometimes reduced to 0)
// ratio gives the position int the path, starting at 0 to waypoints.size()
// ratio = 0: waypoint[0]
// ratio = 1: waypoint[1]
// ...
Eigen::Isometry3d getBlendedPathPoint( double ratio, const std::vector<Eigen::Isometry3d>& waypoints, double blend_radius) {

	int no_of_waypoints = waypoints.size();
	double local_ratio = fmod(ratio, 1.0);
	int l_idx = floor(ratio);
	ROS_ASSERT(l_idx < no_of_waypoints);

	int r_idx = l_idx +1;
	bool has_left_support = l_idx > 0;
	bool has_right_support = r_idx < no_of_waypoints-1;

	Eigen::Isometry3d left = waypoints[l_idx];
	Eigen::Isometry3d right = waypoints[r_idx];
	Eigen::Isometry3d leftleft;
	Eigen::Isometry3d rightright;
	if (has_left_support) {
		leftleft = waypoints[l_idx-1];
	}
	if (has_right_support) {
		rightright = waypoints[r_idx+1];
	}

	// smooth the ratio:
	// ratio_smooth(0) = 0
	// ratio_smooth(1) = 1
	// ratio_smooth(0)' = 0
	// ratio_smooth(1)' = 0
	// looks like this:
	// 1     ---
    //      /
	// 0---/
    //  0     1
	double local_ratio_smooth = (1-cos(local_ratio*M_PI))/2.0;

	// compute the unblended point
	Eigen::Isometry3d unblended;
	unblended.translation() = left.translation() + (right.translation() - left.translation()) * local_ratio ;
	unblended.linear() = Eigen::Quaterniond (left.rotation()).slerp(local_ratio, Eigen::Quaterniond (right.rotation())).toRotationMatrix();
	Eigen::Isometry3d result = unblended;
	double len  = Eigen::Vector3d(right.translation()-left.translation()).norm();

	// see if we have to blend it on either side
	if (has_left_support) {
		double distance_to_left  = Eigen::Vector3d(left.translation()-unblended.translation()).norm();
		double leftleft_len  = Eigen::Vector3d(leftleft.translation()-left.translation()).norm();
		if (leftleft_len/2 < blend_radius)
			blend_radius = leftleft_len/2;
		if (len/2 < blend_radius)
			blend_radius = len/2;
		if ((blend_radius > Minibot::epsilon) && (distance_to_left<blend_radius)) {
			// const Eigen::Vector3d p1 = left.translation() + Eigen::Vector3d(leftleft.translation()-left.translation()).normalized()*blend_radius;
			// const Eigen::Vector3d p2 = left.translation() + Eigen::Vector3d(right.translation()-left.translation()).normalized()*blend_radius;
			Eigen::Vector3d p1 = intersectVectorCircle(left.translation(), leftleft.translation(), blend_radius);
			Eigen::Vector3d p2 = intersectVectorCircle(left.translation(), right.translation(), blend_radius);


			const Eigen::Vector3d pm = (p1+p2)/2.0;

			// Lemma of Euklid
			double h = Eigen::Vector3d(p1-pm).norm();
			double p = Eigen::Vector3d(pm-left.translation()).norm();
			double q = h*h/p;

			// compute centre of circle
			// Eigen::Vector3d centre = left.translation() + Eigen::Vector3d(pm - left.translation()).normalized()*(p+q);
			Eigen::Vector3d centre = intersectVectorCircle(left.translation(), pm, p+q);

			// compute angle between vectors
			double full_angle = getTriangleAngle(leftleft.translation(), left.translation(), right.translation());

			double angle = (1.0-distance_to_left/blend_radius)*full_angle/2;
			Eigen::Vector3d x = Eigen::Vector3d(p2-centre);
			Eigen::Vector3d y = Eigen::Vector3d(left.translation() - p2).normalized()*blend_radius;

			result.translation() = centre + x*cos(angle) + y*sin(angle);
		}
	}
	if (has_right_support) {
		double distance_to_right = Eigen::Vector3d(right.translation()-unblended.translation()).norm();
		double rightright_len  = Eigen::Vector3d(rightright.translation()-right.translation()).norm();
		if (rightright_len/2 < blend_radius)
			blend_radius = rightright_len/2;
		if (len/2 < blend_radius)
			blend_radius = len/2;

		if ((distance_to_right<blend_radius) && (blend_radius > Minibot::epsilon)) {
			// const Eigen::Vector3d p2 = right.translation() + (left.translation()-right.translation()).normalized()*blend_radius;
			// const Eigen::Vector3d p1 = right.translation() + (rightright.translation()-right.translation()).normalized()*blend_radius;
			Eigen::Vector3d p2 = intersectVectorCircle(right.translation(), left.translation(), blend_radius);
			Eigen::Vector3d p1 = intersectVectorCircle(right.translation(), rightright.translation(), blend_radius);

			const Eigen::Vector3d pm = (p2+p1)/2.0;

			// Lemma of Euklid
			double h = Eigen::Vector3d(p2-pm).norm();
			double p = Eigen::Vector3d(pm-right.translation()).norm();
			double q = h*h/p;

			// compute centre of circle
			// Eigen::Vector3d centre = right.translation() + Eigen::Vector3d(pm - right.translation()).normalized()*(p+q);
			Eigen::Vector3d centre = intersectVectorCircle(right.translation(), pm, p+q);


			// compute angle between vectors
			double full_angle = getTriangleAngle(left.translation(), right.translation(), rightright.translation());

			double angle = (1.0-distance_to_right/blend_radius)*full_angle/2;
			Eigen::Vector3d x = Eigen::Vector3d(p2-centre);
			Eigen::Vector3d y = Eigen::Vector3d(right.translation() - p2).normalized()*blend_radius;

			result.translation() = centre + x*cos(angle) + y*sin(angle);

		}
	}

	return result;
}


bool getPathPoint( const minibot::MinibotState& prev, double ratio, const std::vector<minibot::MinibotState>& waypoints, minibot::MinibotState& result, double radius) {

	std::vector<Eigen::Isometry3d> waypoints_iso;
	for (int i = 0;i<waypoints.size();i++) {
		Eigen::Isometry3d iso = getIsometry(waypoints[i]);
		waypoints_iso.push_back(iso);
	}
	double local_ratio = fmod(ratio, 1.0);
	double l_idx = floor(ratio);

	minibot::MinibotState left = waypoints[l_idx];
	minibot::MinibotState right;
	if (l_idx +1 < waypoints.size())
		right = waypoints[l_idx+1];

	Eigen::Isometry3d result_iso;
	if (l_idx < waypoints.size()) {
		result_iso = getBlendedPathPoint(ratio, waypoints_iso, radius);
	}
	else {
		// very last waypoint
		result_iso = waypoints_iso.back();
	}

	tf::poseEigenToMsg(result_iso, result.pose.pose);
	minibot::JointStateConfiguration solutions;

	bool ik_ok = Minibot::Kinematics::computeIK(result.pose.pose, prev.joint_state,solutions);
	if (ik_ok) {
		// IK works for the flange only, we need to add the gripper joints explicitely.
		// take care that the joints are listed in the same order like in goal,
		// since later on, all this is concatenated to one trajectory
		for (size_t i = 0;i<solutions.configuration.size();i++) {
			solutions.configuration[i].name.resize(minibot_joint_names.size());
			solutions.configuration[i].position.resize(minibot_joint_names.size());
			for (size_t j = 0;j<minibot_gripper_joint_names.size();j++) {
				std::string joint_name = minibot_gripper_joint_names[j];

				double start_pos = Utils::getJointValue(left.joint_state, joint_name);
				double goal_pos = Utils::getJointValue(right.joint_state, joint_name);
				int idx = Utils::findJoint(right.joint_state, joint_name);

				solutions.configuration[i].name[idx] = joint_name;
				solutions.configuration[i].position[idx]= (1.0-local_ratio)*start_pos + local_ratio*goal_pos;
			}
		}

		result.configuration = solutions.configuration;
		result.joint_state = solutions.configuration[0];
	} else {
		ROS_ERROR_STREAM_NAMED (LOG_NAME, "Minibot::Database::getPathPoint could not compute IK=" );
		pub_msg.publish(Minibot::Utils::createMsg(planner_prefix + err_msg_prefix  + "Cannot do IK for this path"));
	}
	return ik_ok;
}


bool generateTrajectory(const std::vector<minibot::MinibotState>& waypoints,double radius,
	    				std::vector<minibot::MinibotState>& trajectory) {
	bool result = true;
	bool ok = true;
	trajectory.clear();
	minibot::MinibotState prev = waypoints[0];
	for (int i = 0;i<waypoints.size()-1;i++) {
		const int steps= 50; // arbitrary number, later on, trajectory will be sampled again anyhow.
		for (int j = 0;j<steps;j++) {
			minibot::MinibotState result;
			double ratio = i+((float)j)/steps;
			bool ok = getPathPoint(prev, ratio, waypoints, result, radius);
			if (!ok)
				break;
			trajectory.push_back(result);
			prev = result;

		}
	}
	// add starting point of that bit
	trajectory.push_back(waypoints[waypoints.size()-1]);


	return ok;
}


bool planCartesianBlendedPath(const std::vector<minibot::MinibotState>& waypoints, double blend_radius, trajectory_msgs::JointTrajectory& result_local_traj) {
	trajectory_msgs::JointTrajectory local_local_traj;
	std::vector<minibot::MinibotState> trajectory;
	for (int i = 0;i<waypoints.size();i++) {
		geometry_msgs::Pose pose= waypoints[i].pose.pose;
	    ROS_DEBUG_STREAM_NAMED(LOG_NAME, "Plan(pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z << ")"
					   << " ori=(x=" << pose.orientation.x << " y=" << pose.orientation.y << " z=" << pose.orientation.z << " w=" << pose.orientation.w <<") ");

	}

	bool ok = generateTrajectory(waypoints,blend_radius, trajectory);


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
	if (ok && !success) {
		ROS_ERROR_STREAM_NAMED (LOG_NAME, "Minibot::Planner::planCartesianPath could not compute trajectory timeing" );
		pub_msg.publish(Minibot::Utils::createMsg(planner_prefix + err_msg_prefix  + "Cannot do timing for this path"));
		ok = false;
	}

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
	bool smooth_ok = smoothness_filter.update(local_local_traj, result_local_traj);

	if (ok && !smooth_ok) {
		ROS_ERROR_STREAM_NAMED (LOG_NAME, "Minibot::Planner::planCartesianPath could not smoothe trajectory " );
		pub_msg.publish(Minibot::Utils::createMsg(planner_prefix + err_msg_prefix  + "Cannot smooth trajectory"));
		ok = false;
	}
	return ok;
};

}
}

