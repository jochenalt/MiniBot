#!/usr/bin/env python

import rospy
import copy

import core
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA

from moveit_msgs.msg import DisplayTrajectory
from constants import Constants
from threading import Timer

from minibot.msg import ClientAction

server = None
interactiveMarker = None
lastMarkerPose = None           # last Marker Pose published (necessary, since Marker also publishes no changes )

# returns true if two geometry_msgs/Pose are different (with a little wiggle room)
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

planType2TrajctoryLen = {}

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
    global gearWheelPosePublisher,lastMarkerPose,clientActionPublisher
    
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
        clientAction = ClientAction()
        clientAction.type = ClientAction.ACTIVATE_STATEMENT
        clientAction.statement_block_no = int(marker.marker_name.split("-",1)[1])
        clientActionPublisher.publish(clientAction)
    elif marker.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        if (lastMarkerPose is None or poseIsDifferent(lastMarkerPose, marker.pose)):
            rospy.loginfo( s + ": pose changed")
            gearwheelPose = PoseStamped()
            gearwheelPose.header.frame_id = 'base_link'
            gearwheelPose.pose = copy.copy(marker.pose)
            lastMarkerPose = copy.copy(marker.pose)
            inCallback = 1;
            gearWheelPosePublisher.publish(gearwheelPose);
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

# called whenever the tcp (PoseStamped) changes
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

# called whenever a local DisplayTrajectory is published, draws the trajectory with colorfull little balls
def displayLocalTrajectoryCallback(displayTrajectory):
    makeTrajectoryMarker(displayTrajectory, False, "localtrajectory")

# called whenever a global DisplayTrajectory is published, draws the trajectory with colorfull little balls
def displayGlobalTrajectoryCallback(displayTrajectory):
    makeTrajectoryMarker(displayTrajectory, True, "globaltrajectory")


# compute a nice color for trajectories. 
# with every succeeding number the  main color changes 
def globalTrajectoryColor(no):
    color = ColorRGBA()
    pendulung = no/7.0 % 1.0
    color.r = 0.2
    color.g = 1-no/4 if (no % 2) == 0 else no/4
    color.b = 1-no/4 if (no % 2) == 1 else no/4
    color.a = 1.0

    return color

# create little balls along a trajectory of a pose list 
def makeTrajectoryMarker( displayTrajectory, isGlobal, markerName):
    global server, planType2TrajctoryLen

    # add the sphere list
    counter = 1
    rospy.loginfo("makeTrajectoryMarker {0}, {1}".format(markerName, isGlobal))
    for traj in displayTrajectory.trajectory:
        trajectoryName = markerName + "-" + str(counter)
        server.erase(trajectoryName)

        intMarkerGlobalTrajectory = InteractiveMarker()
        intMarkerGlobalTrajectory.header.frame_id = "base_link"
        intMarkerGlobalTrajectory.scale = 0.01
        intMarkerGlobalTrajectory.name =trajectoryName

        # insert a sphere list control, currently empty
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON

        pointCounter = 1
        marker = None
        for p in traj.joint_trajectory.points:
            if (pointCounter == 1) or (pointCounter == len(traj.joint_trajectory.points)):
                marker = Marker()
                marker.type = Marker.SPHERE_LIST
                if isGlobal:
                    marker.color = globalTrajectoryColor(counter)
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.008
                else:
                    marker.color = globalTrajectoryColor(0)
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.005

                control.markers.append( marker )                
            else:
                if pointCounter == 2:
                    marker = Marker()
                    marker.type = Marker.SPHERE_LIST
                    if isGlobal:
                        marker.scale.x = marker.scale.y = marker.scale.z = 0.004
                        marker.color = globalTrajectoryColor(counter)
                    else:
                        marker.scale.x = marker.scale.y = marker.scale.z = 0.005
                        marker.color = globalTrajectoryColor(0)

                    control.markers.append( marker )                

            jointState = JointState()
            jointState.name =  traj.joint_trajectory.joint_names
            jointState.position =  p.positions
            stampedTcp = core.computeFK(jointState)
            marker.points.append(stampedTcp.pose.position)
            pointCounter = pointCounter + 1

        intMarkerGlobalTrajectory.controls.append( control )
        counter = counter + 1
        server.insert(intMarkerGlobalTrajectory, callbackMarkerChange)

    # delete all remains of previous trajectories
    if markerName in planType2TrajctoryLen:
        for idx in range(counter-1, planType2TrajctoryLen[markerName]):
            trajectoryName = markerName + "-" + str(counter)
            server.erase(trajectoryName)
            counter = counter + 1

    planType2TrajctoryLen[markerName] = len(displayTrajectory.trajectory)

    server.applyChanges();
 


if __name__=="__main__":
    global gearWheelPosePublisher, clientActionPublisher
    rospy.init_node("markers")

    core.initialize()
    server = InteractiveMarkerServer("markers")

    # listen to trajectory update
    #rospy.Subscriber('/move_group/display_planned_path/tcp', PoseArray, displayLocalTrajectoryCallback, queue_size=1);
    rospy.Subscriber('/minibot/local_plan', DisplayTrajectory, displayLocalTrajectoryCallback, queue_size=1);

    # listen to trajectory update
    rospy.Subscriber('/minibot/global_plan', DisplayTrajectory, displayGlobalTrajectoryCallback, queue_size=1);

    # listen to tcp update
    rospy.Subscriber('/tcp/update', PoseStamped, queueUpUpdate, queue_size=1);

    # publish the new position of the marker
    gearWheelPosePublisher = rospy.Publisher('/tcp/gearwheel/update',PoseStamped, queue_size=1)

    # publish new actions
    clientActionPublisher   = rospy.Publisher('/client_action',ClientAction, queue_size=1)

    # create gearwheel at origin, update happens right afterwards
    position = Point(0.0, 0.0, 0.0)
    make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, position, False, False)

    server.applyChanges()

    rospy.loginfo("minibot marker node initialized")

    rospy.spin()

