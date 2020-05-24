#!/usr/bin/env python


import sys
import copy
import rospy
import array
import timeit

# threading model
import threading 
from threading import Lock

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
  plan = None

statements = []                 # memory cache of all programme statements
robotstates = []                # memory cache of all poses

groupArm = None                 # group of all arm links

robot = None                    # RobotCommander Object
globalPlan = [];                # entire plan of the full programme 
displayLocalPlanStartID = None  # start id of local plan to be displayed

configuration = None            # all settings

globalPlanningLock = Lock()             # lock to ensure that global planning is executed single threaded
latestGlobalPlanningThreadID = None     # latest request of global planning that is used to ditch planning requests that are obsolete 



def init():
  global groupArm, robot,plans,\
         displayGlobalPlanPublisher,   \
         displayLocalPlanPublisher

  #  initialize moveit pyhton interface, needs to happenn before .init_node
  moveit_commander.roscpp_initialize(sys.argv)

 
  rospy.init_node('planning')

  # RobotCommander is the interface to the robot as a whole
  robot = moveit_commander.RobotCommander()

  # MoveGroupCommander is the interface to one group of joints.  
  groupArm = moveit_commander.MoveGroupCommander(Constants.MINIBOT_GROUP)
 
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
  groupArm.plan()

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

  (configuration, meta) = db.query_named("mysettings", Configuration._type)
  if configuration is None:
    configuration = Configuration()
    configuration.theme = "cyborgTheme"
    configuration.angle_unit = Configuration.ANGLE_UNIT_RAD
    configuration.save_after 
    configuration.save_after.secs = 3000
    configuration.save_after.nsecs = 0

    db.insert_named("mysettings",configuration)

  

def handleDatabaseAction (request):
  global statements, robotstates, latestGlobalPlanningThreadID
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
    # take care that planning happens only single threaded and also 
    # dont execute all planning requests that are succeeded by another planning requests

    # store thread id of this  request  
    latestGlobalPlanningThreadID = threading.currentThread().ident
    if globalPlanningLock.locked():
      rospy.loginfo("global planning in progress, waiting")

    globalPlanningLock.acquire()
    # if a younger  request has set latestGlobalPlanningThreadID in another thread, quit without doing anything
    if latestGlobalPlanningThreadID == threading.currentThread().ident:
      rospy.loginfo("store and plan")
      db.update_named("default_programme", request.programme_store)
      statements = request.programme_store.statements
      createGlobalPlan()
      rospy.loginfo("display global plan")
      displayGlobalPlan()
      rospy.loginfo("storing, planning, and showing done")
    else:
      rospy.loginfo("later request exists, ignore this one")
    globalPlanningLock.release()

    rospy.loginfo("global planning done ")

  if request.type == DatabaseRequest.READ_SETTINGS:
    (configuration, meta) = db.query_named("mysettings",Configuration._type)
    if configuration:
      response.configuration = configuration
    else:
      rospy.logerr("settings are not initialized");

  if request.type == DatabaseRequest.WRITE_SETTINGS:
    db.update_named("mysettings", request.configuration)
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
    tfs = lastTraj.time_from_start
    tfs.secs =  tfs.secs + deltaTime.secs
    tfs.nsecs =  tfs.nsecs + deltaTime.nsecs
    if tfs.nsecs > 999999999:
       tfs.secs = tfs.secs + 1
       tfs.nsecs = tfs.nsecs - 1000000000

  for mPoint in trajB.multi_dof_joint_trajectory.points:
    result.multi_dof_joint_trajectory.points.append(copy.deepcopy(mPoint))
    lastTraj = result.multi_dof_joint_trajectory.points[-1]
    tfs = lastTraj.time_from_start
    tfs.secs =  tfs.secs + deltaTime.secs
    tfs.nsecs =  tfs.nsecs + deltaTime.nsecs
    if tfs.nsecs > 999999999:
       tfs.secs = tfs.secs + 1
       tfs.nsecs = tfs.nsecs - 1000000000

  return result


# creates a plan of the entire programme
def createGlobalPlan():
  global statements, globalPlan

  startID = None
  globalPlan = []
  foundWaypoint = False

  # find all waypoint blocks, identifiy start and end number and call creation of a loca plan in that interval
  for idx in range(0,len(statements)):
    #print("gp: 0-{0}:{1} type:{2}".format(len(statements)-1, idx, statements[idx].type ))
    isLastStatement = (idx == len(statements)-1)
    # first wyaypoint?
    if startID is None and statements[idx].type == Statement.STATEMENT_TYPE_WAYPOINT:
      startID = idx
      foundWaypoint = True
    # end of a waypoint block? 
    elif not foundWaypoint and statements[idx].type == Statement.STATEMENT_TYPE_WAYPOINT:
      foundWaypoint = True
    # end of a waypoint block? 
    elif startID is not None and foundWaypoint and (statements[idx].type != Statement.STATEMENT_TYPE_WAYPOINT or isLastStatement):
      endID = idx-1
      if isLastStatement:
        endID = idx
      # we found a local sequence of waypoints
      localPlan = createLocalPlan(startID, endID)
      globalPlanItem =  GlobalPlanItem()
      globalPlanItem.start_id = startID
      globalPlanItem.end_id = idx-1
      globalPlanItem.plan = localPlan
      globalPlan.append(globalPlanItem)

      # be ready for the next plan
      startID = endID
      foundWaypoint = False



def displayGlobalPlan():
  global displayGlobalPlanPublisher,globalPlan,configuration
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  if configuration.vis_global_plan:
    for idx in range(0,len(globalPlan)):
      display_trajectory.trajectory.append(globalPlan[idx].plan)

  displayGlobalPlanPublisher.publish(display_trajectory);


# return the next waypoint beginning (and including) startid 
def getNextWaypointID (startid):
  global globalPlan, statements
  for idx in range(startid, len(statements)):
      if statements[idx].type == Statement.STATEMENT_TYPE_WAYPOINT:
        return idx
  return None

# returns the last Waypoint of this block, that ends by EOL or  statement type different from waypoint 
def getLastWaypointID (startid):
  global globalPlan, statements
  for idx in range(startid, len(statements)):
      if statements[idx].type != Statement.STATEMENT_TYPE_WAYPOINT:
        return idx-1
  return len(statements)-1

def getGlobalPlanIDByStatementID(id):
  global globalPlan
  for idx in range(0, len(globalPlan)):
    if globalPlan[idx].start_id <=id and globalPlan[idx].end_id >= id:
      return idx
  return None 

def displayLocalPlan():
  global globalPlan, displayLocalPlanPublisher, configuration, displayLocalPlanStartID
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  if configuration.vis_local_plan and displayLocalPlanStartID != None:
    # look for the local plan in the gobal plan
    startID = getNextWaypointID(displayLocalPlanStartID)
    planID = getGlobalPlanIDByStatementID(startID)
    rospy.loginfo("display local plan from {0} -> startid:{1} planblock:{2}".format(displayLocalPlanStartID,startID, planID))
    display_trajectory.trajectory.append(globalPlan[planID].plan)

  displayLocalPlanPublisher.publish(display_trajectory);


# returns a local plan for all waypoints between startID and endID
def createLocalPlan(startID, endID):
  global statements, groupArm

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
    rospy.loginfo("create local plan from statement {0} to  {1}".format(startID, endID))
    startRS  = getRobotState(statements[startID].pose_uid)
    firstPlan = True
    for idx in range(startID+1, endID+1):
      if statements[idx].type == Statement.STATEMENT_TYPE_WAYPOINT:
        #print("IDX {0}".format(idx))
        endRS = getRobotState(statements[idx].pose_uid)
        robotState.joint_state = copy.copy(startRS.jointState)
        groupArm.set_start_state(copy.copy(robotState))
        groupArm.set_joint_value_target(copy.copy(endRS.jointState))

        rospy.loginfo("create micro plan from statement {0} to  {1}".format(startID, idx))
        if firstPlan:           
          localPlan = groupArm.plan()   # first plan
          firstPlan = False
        else:
          planTmp = groupArm.plan()     # next plan, merge with previous plan
          localPlan = mergeRobotTrajectory(localPlan, planTmp)
        startRS = endRS
        startID = idx

    localPlan= groupArm.retime_trajectory(robot.get_current_state(), localPlan, 1.0)
    return localPlan


# callback when a statement is activated
def handlePlanningAction(request):
  global statements, groupArm, robot,displayGlobalPlanPublisher, displayLocalPlanStartID

  rospy.loginfo("handle planning Action {0}".format (request.type))
  # think positive
  response = PlanningActionResponse()
  response.error_code.val = ErrorCodes.SUCCESS;

  if request.type == PlanningActionRequest.SELECT_LOCAL_PLAN:
    startID = getStatementIDByUID(request.startStatementUID)

    rospy.loginfo("display local plan between {0}-{1}".format (startID, startID));
    if startID == -1:
      rospy.logerr("statement with uid={0} does not exist".format(request.startStatementUID) )
      response.error_code.val = ErrorCodes.UNKNOWN_STATEMENT_UID
      return response

    displayLocalPlanStartID = startID
    displayLocalPlan()

  if request.type == PlanningActionRequest.CLEAR_PLAN:
    rospy.loginfo("clear plan");
    displayLocalPlanStartID = None
    displayLocalPlan()

  if request.type == PlanningActionRequest.SIMULATE_PLAN:
    rospy.loginfo("simulate plan");
    startID = getStatementIDByUID(request.startStatementUID)
    if startID != None:
      planID = getGlobalPlanIDByStatementID(startID)
      if  planID != None:
        groupArm.execute(globalPlan[planID].plan, wait=True)

  if request.type == PlanningActionRequest.GLOBAL_PLAN:
    rospy.loginfo("create global plan");
    createGlobalPlan()
    displayGlobalPlan()

  if request.type == PlanningActionRequest.VIS_GLOBAL_PLAN:
    configuration.vis_global_plan = request.jfdi
    rospy.loginfo("visualize global plan: {0}".format(configuration.vis_global_plan))
    displayGlobalPlan()

  if request.type == PlanningActionRequest.VIS_LOCAL_PLAN:
    configuration.vis_local_plan = request.jfdi
    rospy.loginfo("visualize local plan: {0}".format(configuration.vis_local_plan))
    displayLocalPlan()

  return response


if __name__=='__main__':
  try:
    init()
    rospy.loginfo("minibot planning node initialized")

    rospy.spin()
  except rospy.ROSInterruptException:
    pass

