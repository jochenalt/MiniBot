#!/usr/bin/env python


import sys
import copy
import rospy

# import mongodb
import mongodb_store_msgs.srv as db_srv
import mongodb_store.util as db_util
from mongodb_store.message_store import MessageStoreProxy

# import moveit stuff
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# import minibot stuff
import constants
from constants import Constants
from minibot.msg import Statement
from minibot.msg import Programme
from minibot.msg import ErrorCodes
from minibot.msg import Action
from minibot.srv import SetProgramme, SetProgrammeRequest, SetProgrammeResponse
from minibot.srv import PlanningAction, PlanningActionRequest, PlanningActionResponse


statements = []   # all programme statement as recently sent over via set_proramme
group = None      # MoveGroupCommander Object
robot = None      # RobotCommander Object
scene = None      # Planing scene Object

def init():
  global group,robot,display_trajectory_publisher, scene

  #  initialize moveit pyhton interface
  moveit_commander.roscpp_initialize(sys.argv)

  rospy.init_node('planning', anonymous=True)


  # RobotCommander is the interface to the robot as a whole
  robot = moveit_commander.RobotCommander()

  # MoveGroupCommander is the interface to one group of joints.  
  group = moveit_commander.MoveGroupCommander(Constants.KINEMATICS_GROUP)

  # instantiate a planning scene
  scene = moveit_commander.PlanningSceneInterface()

  ## publisher of trajectories to be displayed
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=1)


  # errors and messages
  msgErrorPub  = rospy.Publisher('/messages/err',String, queue_size=1)
  msgInfoPub  = rospy.Publisher('/messages/info',String, queue_size=1)
  msgWarnPub  = rospy.Publisher('/messages/warn',String, queue_size=1)

  # minibot planning services
  set_programme = rospy.Service('set_programme', SetProgramme, handleSetProgramme)
  planning_action = rospy.Service('planning_action', PlanningAction, handlePlanningAction)

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
  #print(request)

  if request.programme.statements:
    statements = request.programme.statements

  if request.action.type == Action.PLAN_PATH:
    rospy.loginfo("Action.PLAN_PATH")
  
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

    robotStartState = RobotState()
    robotStartState.joint_state.header = Header()
    robotStartState.joint_state.header.stamp = rospy.Time.now()

    # compute waypoints, but leave out the first one, which is the start state
    group.clear_pose_targets()
    waypoints = []

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    if statements[startID].cartesic_path:
      robotStartState.joint_state = copy.copy(statements[startID].jointState)
      group.set_start_state(copy.copy(robotStartState))
      for idx in range(startID+1, endID+1):
        waypoints.append(copy.copy(statements[idx].pose))
      (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
    else:
      for idx in range(startID+1, endID+1):
        robotStartState.joint_state = copy.copy(statements[idx-1].jointState)
        group.set_start_state(copy.copy(robotStartState))
        group.set_pose_target(statements[idx].pose)
        plan = group.plan()
        display_trajectory.trajectory.append(plan)

      ## visualization is done by plan/compute cartesian path
      display_trajectory_publisher.publish(display_trajectory);

      ## visualization is done by plan/compute cartesian path
      display_trajectory_publisher.publish(display_trajectory);

  if request.action.type == Action.CLEAR_PATH:
    rospy.loginfo("Action.CLEAR_PATH")
    # dont know how to delete a plan and remove the displayed trajectory!?
    # so set start and end point to current position
    group.clear_pose_targets()
    group.set_start_state_to_current_state()
    group.set_pose_target(group.get_current_pose().pose)
    plan = group.plan()

  return response


if __name__=='__main__':
  try:

    global msgErrorPub,msgInfoPub, msgWarnPub

    init()


    rospy.spin()
  except rospy.ROSInterruptException:
    pass

