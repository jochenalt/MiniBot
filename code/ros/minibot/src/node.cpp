
#include "mongodb_store/message_store.h"
#include <boost/foreach.hpp>

#include <sstream>
#include <cassert>

using namespace mongodb_store;
using namespace std;


#include <ros/ros.h>

#include "utils.h"
#include "node.h"
#include "kinematics.h"
#include "dispatcher.h"
#include "database.h"
#include "marker.h"
#include "planner.h"
#include "constants.h"


// publisher of new joint values
ros::Publisher pub_joint_state_uiui;

// publisher of possible configurations
ros::Publisher pub_joint_values_config;

// publisher for new tcp data
ros::Publisher pub_tcp_ui;

// publisher for new joint_states
ros::Publisher pub_joint_state_ui;

// publisher for joint_states
ros::Publisher pub_joint_state;

// publisher for messages
ros::Publisher pub_msg;


int main(int argc, char *argv[]) {
	try {

		ros::init(argc, argv, "minibot_server");
		ros::NodeHandle nh;
		ROS_INFO_STREAM("starting minibot server node");

		Minibot::Utils::init();
		Minibot::Planner::init();
		Minibot::Kinematics::init();
		Minibot::Gearwheel::init();
		Minibot::Database::init();

		// publish messages to UI
		pub_msg = nh.advertise<std_msgs::String>("/msg", 10);

		// listen to changes of the tcp coming from UI
		ros::Subscriber pose_input = nh.subscribe("/pose/input/update", 10,Minibot::Dispatcher::updateTCPCallback);

		// listen to changes of the joints coming from UI
		ros::Subscriber joint_states_input = nh.subscribe("/joint_states/input/update", 10, Minibot::Dispatcher::updateJointStatesCallback);

		// listen to changes of the joints coming from UI
		ros::Subscriber configuration_input = nh.subscribe("/joint_configuration/input/update", 10, Minibot::Dispatcher::updateJointStatesConfigurationCallback);

		// publish new joints (forwarded by joint_state_publisher)
		pub_joint_state_ui = nh.advertise<sensor_msgs::JointState>("/joint_states/update", 10);

		// publish new tcp data, consumed by UI
		pub_tcp_ui = nh.advertise<minibot::MinibotPose>("/pose/update", 10);

		// publish new joint states, consumed by UI
		pub_joint_values_config = nh.advertise<minibot::JointStateConfiguration>("/joint_states/configuration", 10);

		// publish new joint states, consumed by UI
		pub_joint_state = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

		// database service
		ros::ServiceServer database_srv = nh.advertiseService("database", Minibot::Database::handleDatabaseAction);

		ros::AsyncSpinner spinner(0); // 0 = one thread per core
		spinner.start();

		// carry out a loop that publishs the joint_states at 20Hz
		ros::Rate loop_rate(joint_state_publish_rate);
		uint32_t joint_states_seq = 0;
		while(ros::ok()) {

			// joint state publisher: take the most recent joint_state and publish it to /joint_states
			minibot::MinibotState state = Minibot::Kinematics::getLastMinibotState();
			state.joint_state.header.stamp = ros::Time::now();
			state.joint_state.header.seq = joint_states_seq++;
			pub_joint_state.publish(state.joint_state);

			loop_rate.sleep();
		}

		ros::waitForShutdown();
		ROS_INFO_STREAM("shutting down minibot server node");

	} catch (ros::Exception &e) {
		ROS_ERROR("Error occured: %s ", e.what());
	}
	return 0;

}

