

#ifndef MININBOT_KINEMATICS_H_
#define MIINBOT_KINEMATICS_H_

#include "ros/ros.h"

#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <minibot/GetPositionAllIK.h>
#include <minibot/GetPositionAllFK.h>
#include <minibot/JointStateConfiguration.h>
#include <minibot/MinibotPose.h>
#include <minibot/MinibotState.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "utils.h"

namespace Minibot {
  namespace Kinematics {

    // compute all pose values to a given pose. Does not consider joint limits
    // returns true if IK was successful.
    geometry_msgs::Pose computeTCPBase(const geometry_msgs::Pose& tcpPose, double tool_distance);
    geometry_msgs::Pose computeTCPTip(const geometry_msgs::Pose& flangePose, double tool_distance);

    // compute all pose values to a given pose. Does not consider joint limits
    // returns true if IK was successful.
    bool computeIK(const geometry_msgs::Pose& pose, const sensor_msgs::JointState& jointState, minibot::JointStateConfiguration& solutions);

    // computes the pose out of given joint values
    void computeFK(const sensor_msgs::JointState& jointState, geometry_msgs::Pose& pose);


    // check if JointState is within robot limits
    // use a cached kinematic_state
    bool satisfiesBounds(const robot_state::RobotStatePtr& kinematic_state,const sensor_msgs::JointState& joint_state);

    // check if the robotstate is in self-collision
    bool inSelfCollision(const robot_state::RobotStatePtr& kinematic_state);

    // call me before anything else, but after node initialisation
    void init();

    // add or overwrite the end effector positions from eff to joint_state
    bool setEndEffectorPosition(sensor_msgs::JointState& joint_state, const sensor_msgs::JointState& eff);

    // the current state state to be published to /joint_states
    // synchronized method
    void setLastMinibotState(const minibot::MinibotState& joint_state);
    minibot::MinibotState getLastMinibotState();


  }
}


#endif
