#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander



import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from minibot.msg import Statement
from minibot.msg import Programme
from minibot.msg import ErrorCodes
from minibot.msg import Action

from minibot.srv import SetProgramme, SetProgrammeRequest, SetProgrammeResponse
from minibot.srv import PlanningAction, PlanningActionRequest, PlanningActionResponse

import constants
from constants import Constants

statements = []   # all programme statement as recently sent over via set_proramme
group = None      # MoveGroupCommander Object
robot = None      # RobotCommander Object

def init():
  global group,robot,display_trajectory_publisher


  #  initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('planning',
                  anonymous=True)

  # Instantiate a RobotCommander object.  This object is an interface to
  # the robot as a whole.
  robot = moveit_commander.RobotCommander()

  # Instantiate a MoveGroupCommander object.  This object is an interface
  # to one group of joints.  In this case the group is the joints in the left
  # arm.  This interface can be used to plan and execute motions on the left
  # arm.
  group = moveit_commander.MoveGroupCommander(Constants.KINEMATICS_GROUP)

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=1)

  rospy.loginfo("planning node initialized")


def getStatementIDByUID(uid):
  #print("getStatementIDByUID(" + str(uid) + ")")
  idx = 0;
  for stmt in statements:
    #print("stmt(: " + str(idx) + ")=" + str(stmt.uid))
    if stmt.uid == uid:
        #print("-> " + str(idx));
        return idx
    idx = idx + 1
  #print("-> not found -1");
  return -1


def handleSetProgramme(request):
  global statements
  statements = request.programme.statements
  #print ("start handleSetProgramme")
  #print (statements)
  #print ("end handleSetProgramme")
  response = SetProgrammeResponse()
  response.error_code.val = ErrorCodes.SUCCESS;
  return response


def handlePlanningAction(request):
  global statements, group, robot,display_trajectory_publisher
  #print("handlePlanningAction")
  # think positive
  response = PlanningActionResponse()
  response.error_code.val = ErrorCodes.SUCCESS;
  print(request)

  if request.programme.statements:
    statements = request.programme.statements

  if request.action.type == Action.DISPLAY_PATH:
    #print("in Dispay path")
  
    startID = getStatementIDByUID(request.action.startStatementUID)
    endID = getStatementIDByUID(request.action.endStatementUID)
    if startID == -1:
      rospy.logerr("startStatementUID {0} does not exist".format(request.action.startStatementUID) )
      response.error_code.val = ErrorCodes.UNKNOWN_STATEMENT_UID
      return response
    if endID == -1:
      rospy. logerr("endStatementUID {0} does not exist".format(request.action.endStatementUID))
      response.error_code.val = ErrorCodes.UNKNOWN_STATEMENT_UID
      return response

    #print("start pose")
    startJointState = statements[startID].jointState
    endPose = statements[endID].pose

    robotStartState = RobotState()
    robotStartState.joint_state = startJointState
    robotStartState.joint_state.header = Header()
    robotStartState.joint_state.header.stamp = rospy.Time.now()

    group.set_pose_target(endPose)
    group.set_start_state(robotStartState);

    ## Now, we call the planner to compute the plan
    ## and visualize it if successful
    ## Note that we are just planning, not asking move_group 
    ## to actually move the robot
    plan1 = group.plan()
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    #display_trajectory.trajectory_start = startPose.jointState;
    #display_trajectory.trajectory_start = robot.get_current_state();
    #display_trajectory.trajectory_start = robotStartState;
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory);
    print("plan done")


  return response


if __name__=='__main__':
  try:

    global msgErrorPub,msgInfoPub, msgWarnPub
    init()

    # errors and messages
    msgErrorPub  = rospy.Publisher('/messages/err',String, queue_size=1)
    msgInfoPub  = rospy.Publisher('/messages/info',String, queue_size=1)
    msgWarnPub  = rospy.Publisher('/messages/warn',String, queue_size=1)

    set_programme = rospy.Service('set_programme', SetProgramme, handleSetProgramme)
    planning_action = rospy.Service('planning_action', PlanningAction, handlePlanningAction)

    rospy.spin()
  except rospy.ROSInterruptException:
    pass

