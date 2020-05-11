#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander



import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

from minibot.msg import Statement
from minibot.msg import Programme
from minibot.srv import SetProgramme, SetProgrammeRequest, SetProgrammeResponse
from minibot.msg import ErrorCodes

import constants
from constants import Constants

statements = []

def init():
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

  rospy.loginfo("planning node initialized")

def handleSetProgramme(prg):
  statements = prg
  print("handleSetProgramme")
  print(prg)
  response = SetProgrammeResponse()
  response.error_code.val = ErrorCodes.SUCCESS;
  return response


if __name__=='__main__':
  try:
    init()

    s = rospy.Service('set_programme', SetProgramme, handleSetProgramme)

    rospy.spin()
  except rospy.ROSInterruptException:
    pass

