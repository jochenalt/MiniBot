#include <ros/ros.h>

#include "utils.h"
#include "node.h"
#include "kinematics.h"
#include "dispatcher.h"

#include "planner.h"

// publisher of new joint values
ros::Publisher pub_joint_state_uiui;

// publisher of possible configurations
ros::Publisher pub_joint_values_config;

// publisher for new tcp data
ros::Publisher pub_tcp_ui;

// publisher for new joint_states
ros::Publisher pub_joint_state_ui;

// publisher for messages
ros::Publisher pub_msg;

int main(int argc, char *argv[]) {
	try {

		ros::init(argc, argv, "minibot_server");
		ros::NodeHandle nh;
		ROS_INFO_STREAM("starting minibot server node");

		ros::AsyncSpinner spinner(1); // 0 = one thread per core
		spinner.start();

		Minibot::Utils::init();
		Minibot::Planner::init();
		Minibot::Kinematics::init();

		// publish messages to UI
		pub_msg = nh.advertise<std_msgs::String>("/msg", 10);

		// listen to changes of the tcp coming from UI
		ros::Subscriber sub_tcp_update = nh.subscribe("/pose/input/update", 10,
				Minibot::Dispatcher::updateTCPCallback);

		// listen to changes of the joints coming from UI
		ros::Subscriber sub_joint_states_update = nh.subscribe(
				"/joint_states/input/update", 10,
				Minibot::Dispatcher::updateJointStatesCallback);

		// publish new joints (forwarded by joint_state_publisher)
		pub_joint_state_ui = nh.advertise<sensor_msgs::JointState>(
				"/joint_states/update", 10);

		// publish new tcp data, consumed by UI
		pub_tcp_ui = nh.advertise<minibot::MinibotPose>("/pose/update", 10);

		// publish new joint states, consumed by UI
		pub_joint_values_config =
				nh.advertise<minibot::JointStateConfiguration>(
						"/joint_states/configuration", 10);

		ros::waitForShutdown();
		ROS_INFO_STREAM("shutting down minibot server node");

	} catch (ros::Exception &e) {
		ROS_ERROR("Error occured: %s ", e.what());
	}
	return 0;

}

