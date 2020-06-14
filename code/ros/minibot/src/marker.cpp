/*
 * dispatcher.cpp
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */


#include "ros/ros.h"

#include "marker.h"
#include "node.h"

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

static interactive_markers::InteractiveMarkerServer* server = NULL;

const std::string gearwheel_marker_name = "gearwheel";
void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    	pub_gearwheel_pose.publish(feedback->pose);

      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}


void init() {
	  server = new interactive_markers::InteractiveMarkerServer("markers","",true);
	  InteractiveMarker int_marker;
	  int_marker.header.frame_id = "base_link";
	  int_marker.scale = 0.03;
	  int_marker.name = gearwheel_marker_name;

	  InteractiveMarkerControl control;

	  control.always_visible = true;
	  Marker marker;
	  marker.type = Marker::SPHERE;
	  marker.scale.x = int_marker.scale *0.5;
	  marker.scale.y = int_marker.scale *0.5;
	  marker.scale.z = int_marker.scale *0.5;
	  marker.color.r = 1.0;
	  marker.color.g = 0.2;
	  marker.color.b = 0.0;
	  marker.color.a = 1.0;
	  control.markers.push_back( marker );
	  int_marker.controls.push_back( control );
	  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
	  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
	  control.independent_marker_orientation = true;

	  server->insert(int_marker);
	  server->setCallback(int_marker.name, &processFeedback);
}

void updateGerwheelPose (const geometry_msgs::Pose& pose) {
	if (server == NULL)
		ROS_ERROR_STREAM_NAMED(LOG_NAME, "interactive server not initialized");

	bool ok = server->setPose(gearwheel_marker_name, pose);
	if (!ok)
		ROS_ERROR_STREAM_NAMED(LOG_NAME, "marker with name " << gearwheel_marker_name << " does not exist");
	server->applyChanges();
}


}
}
