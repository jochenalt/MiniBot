

#ifndef MININBOT_KINEMATICS_H_
#define MIINBOT_KINEMATICS_H_

#include "ros/ros.h"

#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

#include "utils.h"

namespace Minibot {
  namespace Kinematics {
    // compute all pose values to a given pose. Does not consider joint limits
    // returns true if IK was successful.
    bool compute_ik(const geometry_msgs::Pose& pose, std::vector<sensor_msgs::JointState>& solutions);

    // computes the pose out of iven joint values
    void compute_fk(const sensor_msgs::JointState& jointState, geometry_msgs::Pose& pose);

    // check if JointState is within robot limits
    // use a cached kinematic_state
    bool satisfiesBounds(const robot_state::RobotStatePtr& kinematic_state,const sensor_msgs::JointState& joint_state);

    // check if the robotstate is in self-collision
    bool inSelfCollision(const robot_state::RobotStatePtr& kinematic_state);

    void init();
  }
}


#endif
