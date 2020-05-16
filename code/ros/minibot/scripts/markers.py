#!/usr/bin/env python

import rospy
import copy


from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from constants import Constants
from threading import Timer

server = None
interactiveMarker = None
intMarkerTrajectory = None
lastMarkerPose = None           # last Marker Pose published (necessary, since Marker also publishes no changes )

def poseIsDifferent (a,b):
    return ((abs(a.position.x - b.position.x) > Constants.POSITION_RESOLUTION) or 
            (abs(a.position.y - b.position.y) > Constants.POSITION_RESOLUTION) or 
            (abs(a.position.z - b.position.z) > Constants.POSITION_RESOLUTION) or 
            (abs(a.orientation.x - b.orientation.x) > Constants.ORIENTATION_RESOLUTION) or 
            (abs(a.orientation.y - b.orientation.y) > Constants.ORIENTATION_RESOLUTION) or 
            (abs(a.orientation.z - b.orientation.z) > Constants.ORIENTATION_RESOLUTION))   


mutexTimer = None
mutextLastData = None
mutexBlockUpdate = False
def blockUpdate():
    global mutexBlockUpdate
    mutexBlockUpdate = True

def fireUpdate():
    global mutexTimer,  mutextLastData, mutexBlockUpdate
    mutexTimer = None
    tcpPoseCallback(mutextLastData)
    mutextLastData = None
    mutexBlockUpdate = False


def queueUpUpdate( tcp ):
    global mutexTimer,  mutextLastData, mutexBlockUpdate
    mutextLastData = tcp
    if mutexBlockUpdate == False:
        fireUpdate()
    else:
        if not mutexTimer is None:
           mutexTimer.cancel()

        mutexTimer = Timer(1.0, fireUpdate)
        mutexTimer.start()


def callbackMarkerChange( marker ):
    global pub,lastMarkerPose
    
    blockUpdate()

    s = "callback from marker '" + marker.marker_name
    s += "' / control '" + marker.control_name + "'"

    mp = ""
    if marker.mouse_point_valid:
        mp = " at " + str(marker.mouse_point.x)
        mp += ", " + str(marker.mouse_point.y)
        mp += ", " + str(marker.mouse_point.z)
        mp += " in frame " + marker.header.frame_id

    if marker.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif marker.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        if (lastMarkerPose is None or poseIsDifferent(lastMarkerPose, marker.pose)):
            rospy.loginfo( s + ": pose changed")
            gearwheelPose = PoseStamped()
            gearwheelPose.header.frame_id = 'base_link'
            gearwheelPose.pose = copy.copy(marker.pose)
            lastMarkerPose = copy.copy(marker.pose)
            inCallback = 1;
            pub.publish(gearwheelPose);
            inCallback = 0;

    elif marker.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif marker.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )

    server.applyChanges()



def make6DofMarker( fixed, interaction_mode, position, show_rot  = False, show_trans = True):
    global interactiveMarker;
    interactiveMarker = InteractiveMarker()
    interactiveMarker.header.frame_id = "base_link"
    interactiveMarker.pose.position = position
    interactiveMarker.scale = 0.03
    interactiveMarker.name ="gearwheel"

    # insert a sphere
    # insert a sphere
    control =  InteractiveMarkerControl()
    control.always_visible = True
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = interactiveMarker.scale *0.5
    marker.scale.y = interactiveMarker.scale *0.5
    marker.scale.z = interactiveMarker.scale *0.5
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.0
    marker.color.a = 1.0
    control.markers.append( marker )
    interactiveMarker.controls.append( control )
    interactiveMarker.controls[0].orientation_mode = InteractiveMarkerControl.VIEW_FACING
    interactiveMarker.controls[0].interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    interactiveMarker.controls[0].independent_marker_orientation = True

    if interaction_mode != InteractiveMarkerControl.NONE:    
        if show_trans:
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.name = "move_x"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            interactiveMarker.controls.append(control)

            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.name = "move_y"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            interactiveMarker.controls.append(control)
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.name = "move_z"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            interactiveMarker.controls.append(control)

        if show_rot: 
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 1
            control.orientation.y = 0
            control.orientation.z = 0
            control.name = "rotate_x"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            interactiveMarker.controls.append(control)
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 1
            control.orientation.z = 0
            control.name = "rotate_z"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            interactiveMarker.controls.append(control)
            
            control = InteractiveMarkerControl()
            control.orientation.w = 1
            control.orientation.x = 0
            control.orientation.y = 0
            control.orientation.z = 1
            control.name = "rotate_y"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            if fixed:
                control.orientation_mode = InteractiveMarkerControl.FIXED
            interactiveMarker.controls.append(control)

    changeGearWheelAppearance()

# receive tcp pose
def tcpPoseCallback(tcpPoseStamped):
    result = server.setPose("gearwheel", tcpPoseStamped.pose, tcpPoseStamped.header);
    if result:
        server.applyChanges();
    else:
        rospy.logerr("tcpPoseCallback: marker named gearwheel not found");

def changeGearWheelAppearance():
    global interactiveMarker;
    server.erase("gearwheel")
    server.insert(interactiveMarker, callbackMarkerChange)
    server.applyChanges()


def trajectoryCallback(msgPoseArray):
    # delete the old trajectory
    server.erase("trajectory")
    # and create a new one
    makeTrajectoryMarker(msgPoseArray)
    server.insert(intMarkerTrajectory, callbackMarkerChange)
    server.applyChanges();


def makeTrajectoryMarker( msgPoseArray ):
    global intMarkerTrajectory;
    intMarkerTrajectory = InteractiveMarker()
    intMarkerTrajectory.header.frame_id = "base_link"
    intMarkerTrajectory.scale = 0.01
    intMarkerTrajectory.name ="trajectory"

    # insert a sphere list control, currently empty
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.interaction_mode = InteractiveMarkerControl.NONE

    # add the sphere list
    marker = Marker()
    marker.type = Marker.SPHERE_LIST
    marker.scale.x = 0.005
    marker.scale.y = 0.005
    marker.scale.z = 0.005
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.0
    marker.color.a = 1.0
    for p in msgPoseArray.poses:
        marker.points.append(p.position)

    # add marker to control and control to InteractievMarker
    control.markers.append( marker )
    intMarkerTrajectory.controls.append( control )


if __name__=="__main__":
    global pub;
    rospy.init_node("markers")
    server = InteractiveMarkerServer("markers")

    # listen to trajectory update
    rospy.Subscriber('/move_group/display_planned_path/tcp', PoseArray, trajectoryCallback, queue_size=1);

    # listen to tcp update
    rospy.Subscriber('/tcp/update', PoseStamped, queueUpUpdate, queue_size=1);

    # publish the new position of the marker
    pub = rospy.Publisher('/tcp/gearwheel/update',PoseStamped, queue_size=1)

    # create gearwheel at origin, update happens right afterwards
    position = Point(0.0, 0.0, 0.0)
    make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, position, False, False)

    server.applyChanges()

    rospy.spin()

