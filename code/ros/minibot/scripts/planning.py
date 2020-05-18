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
from minibot.msg import MinibotState
from minibot.msg import PoseStorage
from minibot.srv import PlanningAction, PlanningActionRequest, PlanningActionResponse
from minibot.srv import Database, DatabaseRequest, DatabaseResponse


statements = []   # memory cache of all programme statements
robotstates = []  # memory cache of all poses

groupArm = None       # group of all arm links
groupGripper = None   # group of all gripper links

robot = None      # RobotCommander Object
scene = None      # Planing scene Object
plan  = None      # latest plan

def init():
  global groupArm,groupGripper, robot,plan,\
         display_trajectory_publisher, scene,  \
         msgErrorPub,msgInfoPub, msgWarnPub

  #  initialize moveit pyhton interface, needs to happenn before .init_node
  moveit_commander.roscpp_initialize(sys.argv)

 
  rospy.init_node('planning')

  # RobotCommander is the interface to the robot as a whole
  robot = moveit_commander.RobotCommander()

  # MoveGroupCommander is the interface to one group of joints.  
  groupArm = moveit_commander.MoveGroupCommander(Constants.MINIBOT_GROUP)
  #groupArm.set_end_effector_link("flange_link")
  #groupGripper = moveit_commander.MoveGroupCommander(Constants.GRIPPER_GROUP)

  # instantiate a planning scene
  scene = moveit_commander.PlanningSceneInterface()

  ## publisher of trajectories to be displayed
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=1)


  # clear any existing plans (if moveit is still running from a previous session)
  groupArm.clear_pose_targets()
  groupArm.set_start_state_to_current_state()
  groupArm.set_joint_value_target(groupArm.get_current_joint_values())
  plan = groupArm.plan()

  # errors and messages
  msgErrorPub  = rospy.Publisher('/messages/err',String, queue_size=1)
  msgInfoPub  = rospy.Publisher('/messages/info',String, queue_size=1)
  msgWarnPub  = rospy.Publisher('/messages/warn',String, queue_size=1)

  # minibot planning services
  planning_action = rospy.Service('planning_action', PlanningAction, handlePlanningAction)

  # database service capable of managing the PosePanel and the ProgrammePanel
  database = rospy.Service('database', Database, handleDatabaseAction)

  # in case the database is empty, initialize
  initDatabase()

  rospy.loginfo("minibot planning node initialized")

def initDatabase ():
  global statements, robotstates
  db = MessageStoreProxy()
  (poseStorage, meta) = db.query_named("default_pose_storage",PoseStorage._type)
  if poseStorage is None:
    poseStorage = PoseStorage()
    db.insert_named("default_pose_storage",poseStorage)
  robotstates = poseStorage.states

  (programme, meta) = db.query_named("default_programme", Programme._type)
  if programme is None:
    programme = Programme()
    db.insert_named("default_programme",programme)
  statements = programme.statements
 

def handleDatabaseAction (request):
  global statements, robotstates
  db = MessageStoreProxy()

  response = DatabaseResponse()
  response.error_code.val = ErrorCodes.SUCCESS

  if request.type == DatabaseRequest.READ_POSES:
    (poseStorage, meta) = db.query_named("default_pose_storage",PoseStorage._type)
    if poseStorage:
      response.pose_store = poseStorage
      robotstates = poseStorage.states
    else:
      rospy.logerror("pose storage is not initialized");

  if request.type == DatabaseRequest.READ_PROGRAMME:
    (programme, meta) = db.query_named("default_programme", Programme._type)

    if programme:
      response.programme_store = programme
      statements = programme.statements
    else:
      rospy.logerror("programme storage is not initialized");

  if request.type == DatabaseRequest.WRITE_POSES:
    db.update_named("default_pose_storage",request.pose_store)
    robotstates = request.pose_store.states

  if request.type == DatabaseRequest.WRITE_PROGRAMME:
    db.update_named("default_programme", request.programme_store)
    statements = request.programme_store.statements

  return response;

# return the RobotState (MinibotState.msg) with a given uid
def getRobotState(uid):
  global robotstates, db
  for rs in robotstates:
    if rs.uid == uid:
        return rs
  return None


# return the id of the statement with a given uid
def getStatementIDByUID(uid):
  global statements
  idx = 0;
  for stmt in statements:
    if stmt.uid == uid:
        return idx
    idx = idx + 1
  return -1


# callback when a statement is activated
def handlePlanningAction(request):
  global statements, groupArm, robot,display_trajectory_publisher, plan

  # think positive
  response = PlanningActionResponse()
  response.error_code.val = ErrorCodes.SUCCESS;

  if request.action.type == Action.PLAN_PATH:
  
    startID = getStatementIDByUID(request.action.startStatementUID)
    endID = getStatementIDByUID(request.action.endStatementUID)
    if startID == -1:
      rospy.logerr("statement with uid={0} does not exist".format(request.action.startStatementUID) )
      response.error_code.val = ErrorCodes.UNKNOWN_STATEMENT_UID
      return response
    if endID == -1:
      rospy.logerr("statement with uid={0} does not exist".format(request.action.endStatementUID))
      response.error_code.val = ErrorCodes.UNKNOWN_STATEMENT_UID
      return response

    robotState = RobotState()
    robotState.joint_state.header = Header()
    robotState.joint_state.header.stamp = rospy.Time.now()

    startRS = getRobotState(statements[startID].pose_uid)
    endRS = getRobotState(statements[endID].pose_uid)

    if startRS is None:
      rospy.logerr("pose with uid {0} does not exist".format(statements[startID].pose_uid))
      response.error_code.val = ErrorCodes.UNKNOWN_POSE_UID
      return None
    if endRS is None:
      rospy.logerr("pose with uid {0} does not exist".format(statements[endID].pose_uid))
      response.error_code.val = ErrorCodes.UNKNOWN_POSE_UID
      return None

    # compute waypoints, but leave out the first one, which is the start state
    groupArm.clear_pose_targets()
    waypoints = []

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()

    if statements[startID].cartesic_path:
      robotState.joint_state = startRS.jointState
      groupArm.set_start_state(copy.copy(robotState))
      for idx in range(startID, endID+1):
        waypointRS = getRobotState(statements[idx].pose_uid)
        waypoints.append(copy.copy(waypointRS.pose))

      (plan,fraction) = groupArm.compute_cartesian_path(waypoints,0.01,0)
      if fraction < 1.0:
        rospy.logerr("incomplete plan with fraction {0} ".format(fraction))
    else:
      for idx in range(startID, endID):
        startRS  = getRobotState(statements[idx].pose_uid)
        endRS = getRobotState(statements[idx+1].pose_uid)
        robotState.joint_state = copy.copy(startRS.jointState)
        groupArm.set_start_state(copy.copy(robotState))
        #groupArm.set_pose_target(copy.copy(endRS.pose), "tool0_link")
        groupArm.set_joint_value_target(copy.copy(endRS.jointState))

        plan = groupArm.plan()
        display_trajectory.trajectory.append(plan)

      ## visualization is done by plan/compute cartesian path
    display_trajectory_publisher.publish(display_trajectory);

  if request.action.type == Action.CLEAR_PLAN:
    # dont know how to delete a plan and remove the displayed trajectory!?
    # so set start and end point to current position
    groupArm.clear_pose_targets()
    groupArm.set_start_state_to_current_state()
    groupArm.set_joint_value_target(groupArm.get_current_joint_values())
    plan = groupArm.plan()

  if request.action.type == Action.SIMULATE_PLAN:
    groupArm.execute(plan, wait=True)

  return response


if __name__=='__main__':
  try:
    init()

    rospy.spin()
  except rospy.ROSInterruptException:
    pass

