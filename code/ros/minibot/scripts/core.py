#!/usr/bin/env python

import roslib
import rospy
import sys
import copy
import rospy



from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header

import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import RobotState
from moveit_msgs.msg import DisplayTrajectory

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionIKRequest

from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from srdfdom.srdf import SRDF

from moveit_msgs.msg import MoveItErrorCodes

import constants
from constants import Constants


# robot starts with base_link which equals world
kinematicGroupLinks = [];       # all links of the kinematic group of the arm, as defined in group "minibot_arm"
robot = None                    # urdf robot description

# contains current joint positions [rad] of revoluting and non-mimic joints
jointStates = []
# contains the names of revoluting and non-minic joints in the same order like jointStates
jointNames = []
# contains the according links from the joint in jointNames 
linkNames = []


def initData():
    global robot, jointNames, jointStates,jointURDFMapping
    robot = URDF.from_parameter_server()
    count = 0

    # get all the links from robotdescription
    for joint in robot.joints:
        if joint.type != "fixed" and joint.mimic == None:
            jointNames.append(joint.name)
            jointStates.append(0)
            linkNames.append(joint.name.replace('_joint','_link'))

    #print ("robot_description:")
    #print (robot)
    # fetch srdF to get the links of minibot_arm
    semnaticRobot = SRDF.from_parameter_server()
    #print ("robot_description_semantic:")
    #print (semnaticRobot)
    groupChain = []
    for groups in semnaticRobot.groups:
        if groups.name == Constants.ARM_GROUP:
            groupChain = groups.links
            for links in groups.links:
                kinematicGroupLinks.append(links.name)
    if kinematicGroupLinks == None:
        rospy.logerr("group definition of {0} not found".format(kinematicGroup))

    # done
    rospy.loginfo("minibot core node initialized")


def tcpGearwheelCallback(tcpPoseStamped):
    joints = computeIK(tcpPoseStamped)
    if joints != None:
        jointPositionPub.publish(joints)
        tcpPub.publish(tcpPoseStamped)


def tcpInputCallback(tcpPoseStamped):
    joints = computeIK(tcpPoseStamped)
    if joints != None:
        jointPositionPub.publish(joints)
        tcpPub.publish(tcpPoseStamped)


# callback for joint changes coming from website sliders
# invoke and post forward kinematics
def jointStatesInputCallback(msgJointState):
    tcpPose = computeFK(msgJointState)
    if tcpPose != None:
        jointPositionPub.publish(msgJointState)
        tcpPub.publish(tcpPose)

# callback for joint changes coming from movegroup planning
# invoke and post forward kinematics
def moveGroupJointStatesInputCallback(msgJointState):
    tcpPose = computeFK(msgJointState)
    if tcpPose != None:
        jointPositionPub.publish(msgJointState)
        tcpPub.publish(tcpPose)


# callback for joint changes, store the data globally in jointStates for later use 
# in inverse kinematics as the current position
def jointStatesCallback(msg):
    global jointStates, robot

    # there are more joints than required, select just the 
    # revoluting and non-mimic joints
    usedJointsIdx = 0
    allJointsIdx = 0
    for joint in robot.joints:
        if joint.type != "fixed" and joint.mimic == None:
            jointStates[usedJointsIdx] = msg.position[allJointsIdx]
            usedJointsIdx = usedJointsIdx+1
        allJointsIdx = allJointsIdx+1


def plannedPathCallback(msgDisplayTrajectory):
    global plannedTCPPath

    poseArray = PoseArray()
    # for all contained trajectories
    for trajectory in msgDisplayTrajectory.trajectory:
        jointNames = []
        for jointName in trajectory.joint_trajectory.joint_names:
            jointNames.append(jointName)
        rospy.logdebug ("jointNames:")
        rospy.logdebug(jointNames)

        # work through the path and compute forward kinematics  
        # collect the result in PoseArray
        jointPosition = []
        for point in trajectory.joint_trajectory.points:
            rospy.logdebug ("point:")
            rospy.logdebug( point)
            jointState = JointState()
            jointState.name = copy.copy(jointNames)
            jointState.position = copy.copy(point.positions)
            tcpPose = computeFK (jointState)
            poseArray.poses.append(copy.copy(tcpPose.pose))

    plannedTCPPath.publish(poseArray)


def computeFK (msgJointState):
    global jointNames
    rospy.wait_for_service('compute_fk')
    try:
        service = rospy.ServiceProxy('compute_fk', GetPositionFK)
        request = GetPositionFKRequest()

        # build robot state of current joint positions of kinematic chain 
        kinematicChainIdx = 0
        robotState = RobotState()
        request.fk_link_names = []

        for link in kinematicGroupLinks:
            request.fk_link_names.append(link)
            joint = link.replace("_link", "_joint")

            index = 0
            for j in msgJointState.name:
                if joint == j:
                    robotState.joint_state.name.append(msgJointState.name[index])
                    robotState.joint_state.position.append( msgJointState.position[index])
                index = index + 1    
                
            kinematicChainIdx = kinematicChainIdx + 1
        request.robot_state = robotState

        # header
        header = Header()
        header.frame_id = 'base_link'
        request.header = header

        rospy.logdebug ("computeFK request:")
        rospy.logdebug(request)
        # do the call to /compute_fk
        response = service(request)
        rospy.logdebug("computeFK response:")
        rospy.logdebug(response)
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            return response.pose_stamped[-1]
        else:
           msgErrorPub.publish("forward kinematics failed ({0})".format(response.error_code.val))
           return None
    except rospy.ServiceException, e:
        rospy.logerr("service call compute_fk failed with %s",e)
        msgErrorPub.publish("forward kinematics failed")
        return None

def computeIK (msgTcpPose):
    rospy.wait_for_service('compute_ik')
    service = rospy.ServiceProxy('compute_ik', GetPositionIK)
    request = GetPositionIKRequest()
    request.ik_request.group_name = Constants.ARM_GROUP
    request.ik_request.timeout = rospy.Duration.from_sec(0.0001)

    # inverse kinematics is computed for the first 6 links
    tcpIndex = 6;
    robotState = RobotState()
    robotState.joint_state.name     =  copy.copy(jointNames[0:tcpIndex-1])
    robotState.joint_state.position = copy.copy(jointStates[0:tcpIndex-1])
    request.ik_request.robot_state  = robotState

    rospy.logdebug ("computeIK:")
    rospy.logdebug ("jointNames:%s", jointNames[0:tcpIndex])

    # goal end pose
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = Constants.FIXED_FRAME
    pose_stamped.pose = copy.copy(msgTcpPose.pose)
    request.ik_request.pose_stamped = pose_stamped

    try:
        response = service(request)
        if response.error_code.val == MoveItErrorCodes.SUCCESS:
            return response.solution.joint_state
        else:
            msgErrorPub.publish("inverse kinematics failed ({0})".format(response.error_code.val))
            return None

    except rospy.ServiceException, e:
        rospy.logerr("service call compute_ik failed with %s",e)
        msgErrorPub.publish("inverse kinematics failed")
        return None


if __name__=="__main__":

    global jointPositionPub,tcpPub, msgErrorPub,msgInfoPub, msgWarnPub,plannedTCPPath

    rospy.init_node("core")

    # read robot description and init global variables 
    initData()

    # listen to current joint states
    rospy.Subscriber('/joint_states', JointState, jointStatesCallback, queue_size=1);
  
    # listen to tcp input coming from various sources (webpage/gearwheel, webpage/tcp/sliders, webpage/joint/sliders)
    rospy.Subscriber('/tcp/gearwheel/update', PoseStamped, tcpGearwheelCallback, queue_size=1);
    rospy.Subscriber('/tcp/input/update', PoseStamped, tcpInputCallback, queue_size=1);
    rospy.Subscriber('/joint_states/input/update', JointState, jointStatesInputCallback, queue_size=1);

    # input from moveit planning comes in via this controller
    rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, moveGroupJointStatesInputCallback, queue_size=1);

    # with any input, post updated joints and tcp  
    jointPositionPub = rospy.Publisher('/joint_states/update',JointState, queue_size=1)
    tcpPub  = rospy.Publisher('/tcp/update',PoseStamped, queue_size=1)

    # errors and messages
    msgErrorPub  = rospy.Publisher('/messages/err',String, queue_size=1)
    msgInfoPub  = rospy.Publisher('/messages/info',String, queue_size=1)
    msgWarnPub  = rospy.Publisher('/messages/warn',String, queue_size=1)

    # listen to trajectory poses
    plannedTCPPath = rospy.Publisher('/move_group/display_planned_path/tcp', PoseArray, queue_size=100); # 10 seconds contingency,path planning happens in steps of 0.1s 
    rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, plannedPathCallback);

    # and go
    rospy.spin()

