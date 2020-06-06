#include <ros/ros.h>

#include "utils.h"
#include "kinematics.h"
#include "planner.h"

int main (int argc, char *argv[])
{

  ros::init (argc, argv, "minibot_server");
  ros::NodeHandle nh;
  ROS_INFO_STREAM("starting minibot server node");

  ros::AsyncSpinner spinner (0); // 0 = one thread per core
  spinner.start ();

  Minibot::Utils::init ();
  Minibot::Planner::init ();
  Minibot::Kinematics::init ();

  // provide service compute_all_ik
  ros::ServiceServer compute_all_ik = nh.advertiseService (
      "compute_all_ik", Minibot::Kinematics::compute_all_ik_service);

  geometry_msgs::Pose pose;
  pose.position.x = 0.2591;
  pose.position.y = 0.0661;
  pose.position.z = 0.3046;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.6780955338888799;
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.7349737729470047;

  std::vector<sensor_msgs::JointState> solutions;
  Minibot::Kinematics::compute_ik (pose, solutions);

  Minibot::Kinematics::compute_fk (solutions[3], pose);

  ros::waitForShutdown ();
  ROS_INFO_STREAM("shutting down minibot server node");

  return 0;

}

