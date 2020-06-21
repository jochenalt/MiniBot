/*
 * dispatcher.cpp
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#include "ros/ros.h"

#include "utils.h"
#include "marker.h"
#include "node.h"
#include "dispatcher.h"
#include "planner.h"
#include "kinematics.h"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

namespace Minibot {
namespace Gearwheel {

#define LOG_NAME "marker"
using namespace visualization_msgs;

static interactive_markers::InteractiveMarkerServer *server = NULL;

const std::string gearwheel_marker_name = "gearwheel";
void processFeedback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	std::ostringstream s;
	s << "Feedback from marker '" << feedback->marker_name << "' "
			<< " / control '" << feedback->control_name << "'";

	std::ostringstream mouse_point_ss;
	if (feedback->mouse_point_valid) {
		mouse_point_ss << " at " << feedback->mouse_point.x << ", "
				<< feedback->mouse_point.y << ", " << feedback->mouse_point.z
				<< " in frame " << feedback->header.frame_id;
	}

	switch (feedback->event_type) {
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM(
				s.str() << ": button click" << mouse_point_ss.str() << ".");
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		ROS_INFO_STREAM(
				s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
		break;

	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		Minibot::Dispatcher::updateGearwheelCallback(feedback->pose);

		ROS_INFO_STREAM(
				s.str() << ": pose changed" << "\nposition = " << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z << "\norientation = " << feedback->pose.orientation.w << ", " << feedback->pose.orientation.x << ", " << feedback->pose.orientation.y << ", " << feedback->pose.orientation.z << "\nframe: " << feedback->header.frame_id << " time: " << feedback->header.stamp.sec << "sec, " << feedback->header.stamp.nsec << " nsec");
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
		Minibot::Dispatcher::updateGearwheelCallback(feedback->pose);
		ROS_INFO_STREAM(
				s.str() << ": mouse down" << mouse_point_ss.str() << ".");
		break;
	}

	server->applyChanges();
}

// create the marker at the tip of the arm
void createTipMarker() {
	InteractiveMarker int_marker;
	int_marker.header.frame_id = Utils::getBaseFrameName();
	int_marker.scale = 0.03;
	int_marker.name = gearwheel_marker_name;

	InteractiveMarkerControl control;
	control.always_visible = true;
	control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
	control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
	control.independent_marker_orientation = true;

	Marker marker;
	marker.type = Marker::SPHERE;
	marker.scale.x = int_marker.scale * 0.5;
	marker.scale.y = int_marker.scale * 0.5;
	marker.scale.z = int_marker.scale * 0.5;
	marker.color.r = 1.0;
	marker.color.g = 0.2;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	control.markers.push_back(marker);
	int_marker.controls.push_back(control);
	int_marker.controls[0].orientation_mode =
			InteractiveMarkerControl::VIEW_FACING;
	int_marker.controls[0].interaction_mode =
			InteractiveMarkerControl::MOVE_PLANE;
	int_marker.controls[0].independent_marker_orientation = true;

	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);
}

void deleteTrajectoryMarker(std::string name, int start_index) {
	std::string trajectory_name = name + "-" + std::to_string(start_index);
	server->erase(trajectory_name);
}

std_msgs::ColorRGBA getTrajectoryColor(int& no) {
	std_msgs::ColorRGBA color;
    color.r = 0.2;
    color.g =  (no % 2) == 0?1-no/4:no/4;
    color.b =  (no % 2) == 1?1-no/4:no/4;
    color.a = 1.0;
    no++;

    return color;
}

std_msgs::ColorRGBA getWaypointColor() {
	int no = 0;
	return getTrajectoryColor(no);
}

void createTrajectoryMarker(const trajectory_msgs::JointTrajectory& joint_trajectory, bool is_global, std::string name, int start_index, int& color_no) {
	// delete the previous trajectory with the same name and number
	std::string trajectory_name = name + "-" + std::to_string(start_index);
	InteractiveMarker int_marker;
	int_marker.header.frame_id = Utils::getBaseFrameName();
	int_marker.scale = 0.01;
	int_marker.name = trajectory_name;

	// insert a sphere list control
	InteractiveMarkerControl control;
	control.always_visible = true;
	control.interaction_mode = InteractiveMarkerControl::BUTTON;

	int point_counter = 1;
	Marker marker;
	for (size_t p_idx;p_idx < joint_trajectory.points.size();p_idx++) {
    	// first and last point is one spehere list,
    	// all points in between is another sphere list,
    	// because one sphere list can have only identical spheres
    	// and the first and last sphere is supposed to be bigger
    	if ((p_idx == 1) || (p_idx == joint_trajectory.points.size()-1)) {
    		marker.type = Marker::SPHERE_LIST;
    		if (is_global) {
    			marker.color = getTrajectoryColor(color_no);
	            marker.scale.x = marker.scale.y = marker.scale.z = 0.008;
    		}
	        else {
	        	marker.color = getWaypointColor();
				marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
	        }
           control.markers.push_back( marker );
    	}
        else {
        	if (p_idx == 2) {
                marker.type = Marker::SPHERE_LIST;
                if (is_global) {
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.004;
                    marker.color = getTrajectoryColor(color_no);
                }
                else {
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
                    marker.color = getWaypointColor();
                }
        	}
        	control.markers.push_back( marker );
        }

    	// add the point to the active sphere list
        sensor_msgs::JointState joint_state;
        joint_state.name =  joint_trajectory.joint_names;
        joint_state.position =  joint_trajectory.points[p_idx].positions;
        geometry_msgs::Pose pose;
	    Minibot::Kinematics::computeFK(joint_state,pose);
	    marker.points.push_back(pose.position);

	    int_marker.controls.push_back( control );
	    server->insert(int_marker, &processFeedback);
	}
    server->applyChanges();
}

void init() {
	ROS_INFO_STREAM_NAMED(LOG_NAME, "module marker init");

	server = new interactive_markers::InteractiveMarkerServer("markers", "",true);

	createTipMarker();

}

void updateGerwheelPose(const geometry_msgs::Pose &pose) {
	if (server == NULL)
		ROS_ERROR_STREAM_NAMED(LOG_NAME, "interactive server not initialized");

	bool ok = server->setPose(gearwheel_marker_name, pose);
	if (!ok)
		ROS_ERROR_STREAM_NAMED(LOG_NAME,
				"marker with name " << gearwheel_marker_name << " does not exist");
	server->applyChanges();
}

}
}
