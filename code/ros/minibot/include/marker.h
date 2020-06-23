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
#include <trajectory_msgs/JointTrajectory.h>


namespace Minibot {
namespace Gearwheel {

	// call me before anything else, but after node initialisation
	void construct();

	void init();

	void updateGearwheelPose(const geometry_msgs::Pose& pose);

	// @brief Display a trajectory, i.e. create an interactive marker per trajectory point.
	// Overwrites the previous trajectory
	// @param joint_trajectory the trajectory to be displayed
	// @param is_global global trajectories are displayed with sphere of a different size
	// @param name name of the trajectory ("local" vs "global"), required to identify trajectories and delete old ones
	// @param trajectory_no number to identify the trajectory, used to undisplay old ones
	// @param color_no every trajectories gets a different color, this is to increment the internal color sequence
	void createTrajectoryMarker(const trajectory_msgs::JointTrajectory& joint_trajectory, bool is_global, std::string name, int trajectory_no, int& color_no);

	// @brief undisplay a trajectory previously displayed via createTrajectoryMarker
	// @param name name as passed to createTrajectoryMarker
	// @param start_index index as passed to createTrajectoryMarker
	void deleteTrajectoryMarker(std::string name, int start_index);



}
}


#endif /* INCLUDE_MARKER_H_ */
