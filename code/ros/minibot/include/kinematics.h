

#ifndef MIINBOT_KINEMATICS_H_
#define MIINBOT_KINEMATICS_H_

#include <moveit/robot_model/robot_model.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// name used for logging
#define LOG_NAME "kinematics"

// compute all pose values to a given pose. Does not consider joint limits
// returns true if IK was successful.
bool compute_ik(const geometry_msgs::Pose& pose, std::vector<sensor_msgs::JointState>& solutions);

// computes the pose out of iven joint values
void compute_fk(const sensor_msgs::JointState& jointState, geometry_msgs::Pose& pose);

#endif
