
#include "ros/ros.h"
#include "utils.h"

#include "kinematics.h"

int main(int argc, char * argv[]) {

	ROS_INFO_STREAM("starting minibot node");

	ros::init(argc, argv, "minibot_server");

	initErrorMap();
	geometry_msgs::Pose pose;
	pose.position.x = 0.2591;
	pose.position.y = 0.0661;
	pose.position.z = 0.3046;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.6780955338888799;
	pose.orientation.z = 0.0;
	pose.orientation.w = 0.7349737729470047;

	std::vector<sensor_msgs::JointState> solutions;
	compute_ik(pose, solutions);

	compute_fk(solutions[3], pose);

	// spin with one thread
	ros::spin();
	return 0;

}


