#!/usr/bin/env python


import sys
import copy
import rospy
import array
import timeit

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
from moveit_msgs.msg import RobotTrajectory

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# import minibot stuff
import constants
import core
from constants import Constants
from minibot.msg import Statement
from minibot.msg import Programme
from minibot.msg import ErrorCodes
from minibot.msg import MinibotState
from minibot.msg import PoseStorage
from minibot.msg import Configuration
from minibot.srv import PlanningAction, PlanningActionRequest, PlanningActionResponse
from minibot.srv import Database, DatabaseRequest, DatabaseResponse

class GlobalPlanItem:
  start_id = 0
  end_id = 0
  localPlan = None

statements = []               # memory cache of all programme statements
robotstates = []              # memory cache of all poses

groupArm = None               # group of all arm links
groupGripper = None           # group of all gripper links

robot = None                  # RobotCommander Object
scene = None                  # Planing scene Object
localPlan  = None             # latest plan on a local sequence of waypoints, contains the currently computed plan
globalPlan = [];              # entire plan of the full programme 
configuration = None          # settings

visualizeGlobalPlan = False    # publish the global plan (for marker.py)
visualizeLocalPlan = False    # publish the local plan (for marker.py)

def init():
  global groupArm,groupGripper, robot,plans,\
         displayGlobalPlanPublisher, scene,  \
         displayLocalPlanPublisher

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
    ## publisher of trajectories to be displayed
  displayLocalPlanPublisher = rospy.Publisher(
                                      '/minibot/local_plan',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=1)

  displayGlobalPlanPublisher = rospy.Publisher(
                                      '/minibot/global_plan',
                                      moveit_msgs.msg.DisplayTrajectory, queue_size=1)


  # clear any existing plans (if moveit is still running from a previous session)
  groupArm.clear_pose_targets()
  groupArm.set_start_state_to_current_state()
  groupArm.set_joint_value_target(groupArm.get_current_joint_values())
  localPlan = groupArm.plan()

  # minibot planning services
  planning_action = rospy.Service('planning_action', PlanningAction, handlePlanningAction)

  # database service capable of managing the PosePanel and the ProgrammePanel
  database = rospy.Service('database', Database, handleDatabaseAction)

  # in case the database is empty, initialize
  initDatabase()

  # initiate a global planning process
  rospy.loginfo("creating a global plan")
  createGlobalPlan()
  displayGlobalPlan()



def initDatabase ():
  global statements, robotstates, configuration
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

  (configuration, meta) = db.query_named("settings", Configuration._type)
  if configuration is None:
    configuration = Configuration()
    configuration.theme = "cyborgTheme"
    configuration.angle_unit = Configuration.ANGLE_UNIT_RAD
    db.insert_named("settings",configuration)

  

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
      rospy.logerr("pose storage is not initialized");

  if request.type == DatabaseRequest.READ_PROGRAMME:
    (programme, meta) = db.query_named("default_programme", Programme._type)

    if programme:
      response.programme_store = programme
      statements = programme.statements
    else:
      rospy.logerr("programme storage is not initialized");

  if request.type == DatabaseRequest.WRITE_POSES:
    db.update_named("default_pose_storage",request.pose_store)
    robotstates = request.pose_store.states

  if request.type == DatabaseRequest.WRITE_PROGRAMME:
    db.update_named("default_programme", request.programme_store)
    statements = request.programme_store.statements

  if request.type == DatabaseRequest.READ_SETTINGS:
    (config, meta) = db.query_named("settings",Configuration._type)
    if config:
      response.configuration = config
      configuration = config
    else:
      rospy.logerr("settings are not initialized");

  if request.type == DatabaseRequest.WRITE_SETTINGS:
    db.update_named("settings", request.configuration)
    configuration = request.configuration

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

#  return a merge of two trajectories of type moveit_msgs/RobotTrajectory
# timeing (i.e. time_from_start) is adapted accordingly 
def mergeRobotTrajectory(trajA,trajB):
  result = copy.deepcopy(trajA);
  # check compatibility
  i = 0
  for jointName in trajA.joint_trajectory.joint_names:
    if jointName != trajB.joint_trajectory.joint_names[i]:
      rospy.logerr("cannot merge two incompatible trajectories {0} {1}".format(str(trajA.joint_trajectory.joint_names), trajB.joint_trajectory.joint_names))
      return None  
    i = i+1

  deltaTime = trajA.joint_trajectory.points[-1].time_from_start
  for trajPoint in trajB.joint_trajectory.points:
    result.joint_trajectory.points.append(copy.deepcopy(trajPoint))
    lastTraj = result.joint_trajectory.points[-1]
    lastTraj.time_from_start.secs =  lastTraj.time_from_start.secs + deltaTime.secs
    lastTraj.time_from_start.nsecs =  lastTraj.time_from_start.nsecs + deltaTime.nsecs
    if lastTraj.time_from_start.nsecs > 999999999:
       lastTraj.time_from_start.secs = lastTraj.time_from_start.secs + 1
       lastTraj.time_from_start.nsecs = lastTraj.time_from_start.nsecs - 1000000000

  for mPoint in trajB.multi_dof_joint_trajectory.points:
    result.multi_dof_joint_trajectory.points.append(copy.deepcopy(mPoint))
    lastTraj = result.multi_dof_joint_trajectory.points[-1]
    lastTraj.time_from_start.secs =  lastTraj.time_from_start.secs + deltaTime.secs
    lastTraj.time_from_start.nsecs =  lastTraj.time_from_start.nsecs + deltaTime.nsecs
    if lastTraj.time_from_start.nsecs > 999999999:
       lastTraj.time_from_start.secs = lastTraj.time_from_start.secs + 1
       lastTraj.time_from_start.nsecs = lastTraj.time_from_start.nsecs - 1000000000

  return result

# creates a plan of the entire programme
def createGlobalPlan():
  global statements, globalPlan
  startID = None
  globalPlan = [];
  for idx in range(0,len(statements)):
    if startID is None and statements[idx].type == Statement.STATEMENT_TYPE_WAYPOINT:
      startID = idx
    elif startID is not None and statements[idx].type != Statement.STATEMENT_TYPE_WAYPOINT and idx == startID+1:
      # we have only one waypoint
      startID = None
    elif startID is not None and statements[idx].type != Statement.STATEMENT_TYPE_WAYPOINT and idx > startID+1:
      # we found a local sequence of waypoints
      localPlan = createLocalPlan(startID, idx-1)
      globalPlanItem =  GlobalPlanItem()
      globalPlanItem.start_id = startID
      globalPlanItem.end_id = idx-1
      globalPlanItem.plan = localPlan
      globalPlan.append(globalPlanItem)

      # be ready for the next plan
      startID = None


def displayGlobalPlan():
  global displayGlobalPlanPublisher,globalPlan,configuration
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  if configuration.vis_global_plan:
    for idx in range(0,len(globalPlan)):
      display_trajectory.trajectory.append(globalPlan[idx].plan)

  displayGlobalPlanPublisher.publish(display_trajectory);


def displayLocalPlan():
  global displayLocalPlanPublisher, localPlan, configuration
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  if configuration.vis_local_plan:
    display_trajectory.trajectory.append(localPlan)

  displayLocalPlanPublisher.publish(display_trajectory);


# returns a local plan for all waypoints between startID and endID
def createLocalPlan(startID, endID):
  global statements, groupArm, localPlan

  robotState = RobotState()
  robotState.joint_state.header = Header()
  robotState.joint_state.header.stamp = rospy.Time.now()

  startRS = getRobotState(statements[startID].pose_uid)
  endRS = getRobotState(statements[endID].pose_uid)

  # compute waypoints, but leave out the first one, which is the start state
  groupArm.clear_pose_targets()
  if statements[startID].cartesic_path:
    waypoints = []
    robotState.joint_state = startRS.jointState
    groupArm.set_start_state(copy.copy(robotState))
    for idx in range(startID, endID+1):
      waypointRS = getRobotState(statements[idx].pose_uid)
      waypoints.append(copy.copy(waypointRS.pose))

    (localPlan,fraction) = groupArm.compute_cartesian_path(waypoints,0.01,0)
    if fraction < 1.0:
      rospy.logerr("incomplete plan with fraction {0} ".format(fraction))
  else:
    localPlan = None
    for idx in range(startID, endID):
      startRS  = getRobotState(statements[idx].pose_uid)
      endRS = getRobotState(statements[idx+1].pose_uid)
      robotState.joint_state = copy.copy(startRS.jointState)
      groupArm.set_start_state(copy.copy(robotState))
      groupArm.set_joint_value_target(copy.copy(endRS.jointState))

      if idx == startID:           
        localPlan = groupArm.plan()
      else:
        planTmp = groupArm.plan()
        localPlan = mergeRobotTrajectory(localPlan, planTmp)

    localPlan= groupArm.retime_trajectory(robot.get_current_state(), localPlan, 1.0)

    displayLocalPlan()
    return localPlan


# callback when a statement is activated
def handlePlanningAction(request):
  global statements, groupArm, robot,displayGlobalPlanPublisher, localPlan,visualizeGlobalPlan, visualizeLocalPlan

  rospy.loginfo("handle planning Action {0}".format (request.type))
  # think positive
  response = PlanningActionResponse()
  response.error_code.val = ErrorCodes.SUCCESS;

  if request.type == PlanningActionRequest.PLAN_PATH:
    startID = getStatementIDByUID(request.startStatementUID)
    endID = getStatementIDByUID(request.endStatementUID)
    rospy.loginfo("create local plan between {0}-{1}".format (startID, startID));
    if startID == -1:
      rospy.logerr("statement with uid={0} does not exist".format(request.startStatementUID) )
      response.error_code.val = ErrorCodes.UNKNOWN_STATEMENT_UID
      return response
    if endID == -1:
      rospy.logerr("statement with uid={0} does not exist".format(request.endStatementUID))
      response.error_code.val = ErrorCodes.UNKNOWN_STATEMENT_UID
      return response

    localPlan = createLocalPlan(startID, endID); 

  if request.type == PlanningActionRequest.CLEAR_PLAN:
    rospy.loginfo("clear  plan");
    # dont know how to delete a plan and remove the displayed trajectory!?
    # so set start and end point to current position
    groupArm.clear_pose_targets()
    groupArm.set_start_state_to_current_state()
    groupArm.set_joint_value_target(groupArm.get_current_joint_values())
    localPlan = groupArm.plan()

  if request.type == PlanningActionRequest.SIMULATE_PLAN:
    rospy.loginfo("simulate plan");
    groupArm.execute(localPlan, wait=True)

  if request.type == PlanningActionRequest.GLOBAL_PLAN:
    rospy.loginfo("create global plan");
    createGlobalPlan()
    displayGlobalPlan()

  if request.type == PlanningActionRequest.VIS_GLOBAL_PLAN:
    configuration.vis_global_plan = request.jfdi
    rospy.loginfo("visualize global plan: {0}".format(visualizeGlobalPlan))
    displayGlobalPlan()

  if request.type == PlanningActionRequest.VIS_LOCAL_PLAN:
    configuration.vis_local_plan = request.jfdi
    rospy.loginfo("visualize local plan: {0}".format(visualizeLocalPlan))
    displayLocalPlan()

  return response


if __name__=='__main__':
  try:
    init()
    rospy.loginfo("minibot planning node initialized")

    rospy.spin()
  except rospy.ROSInterruptException:
    pass

