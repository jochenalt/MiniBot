
#include "mongodb_store/message_store.h"
#include <boost/foreach.hpp>

#include <sstream>
#include <cassert>

using namespace mongodb_store;
using namespace std;


#include <ros/ros.h>

#include "globals.h"

#include "utils.h"
#include "kinematics.h"
#include "dispatcher.h"
#include "database.h"
#include "marker.h"
#include "planner.h"
#include "constants.h"
#include "execution.h"


int main(int argc, char *argv[]) {
	try {

		ros::init(argc, argv, "minibot_server");
		ros::NodeHandle nh;
		ROS_INFO_STREAM("starting minibot server node");

		Minibot::Utils::construct();
		Minibot::Kinematics::construct();
		Minibot::Gearwheel::construct();
		Minibot::Database::construct();
		Minibot::Planner::construct();
		Minibot::Execution::construct();

		Minibot::Gearwheel::init();
		Minibot::Planner::init();

		// *** publish messages to UI ***
		Minibot::pub_msg = nh.advertise<std_msgs::String>("/msg", 10);

		// **** topcis handling kinematics ***
		// listen to changes of the tcp coming from UI
		ros::Subscriber pose_input = nh.subscribe("/pose/input/update", 1,Minibot::Dispatcher::updateTCPCallback);

		// listen to changes of the joints coming from UI
		ros::Subscriber joint_states_input = nh.subscribe("/joint_states/input/update", 1, Minibot::Dispatcher::updateJointStatesCallback);

		// listen to changes of the joints coming from UI
		ros::Subscriber joint_states_execution = nh.subscribe("/move_group/fake_controller_joint_states", 1, Minibot::Dispatcher::updateExecutionJointStatesCallback);

		// listen to changes of the joints coming from UI
		ros::Subscriber configuration_input = nh.subscribe("/joint_configuration/input/update", 1, Minibot::Dispatcher::updateJointStatesConfigurationCallback);

		// publish new joints (forwarded by joint_state_publisher)
		Minibot::pub_joint_state_ui = nh.advertise<sensor_msgs::JointState>("/joint_states/update", 1);

		// publish new tcp data, consumed by UI
		Minibot::pub_tcp_ui = nh.advertise<minibot::MinibotPose>("/pose/update", 1);

		// publish new joint states, consumed by UI
		Minibot::pub_joint_values_config = nh.advertise<minibot::JointStateConfiguration>("/joint_states/configuration", 1);

		// publish new joint states, consumed by UI
		Minibot::pub_joint_state = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

		// *** publish messages around planning ***
		Minibot::pub_local_plan = nh.advertise<moveit_msgs::DisplayTrajectory>("/minibot/local_plan",1);
		Minibot::pub_global_plan = nh.advertise<moveit_msgs::DisplayTrajectory>("/minibot/global_plan",1);

		// *** services ***
		// database service, maintaining settings, pose storage and programm storage
		ros::ServiceServer database_srv = nh.advertiseService("/database", Minibot::Database::handleDatabaseAction);

		// planning service, handling planing requests and generating displayed trajectories
		ros::ServiceServer planning_srv = nh.advertiseService("/planning", Minibot::Planner::handlePlanningAction);

		// *** planning ***
		Minibot::Planner::plan();

		ros::AsyncSpinner spinner(0); // 0 = one thread per core
		spinner.start();

		// carry out a loop that publishs the joint_states at 20Hz
		ros::Rate loop_rate(Minibot::joint_state_publish_rate);
		uint32_t joint_states_seq = 0;
		while(ros::ok()) {

			// joint state publisher: take the most recent joint_state and publish it to /joint_states
			minibot::MinibotState state = Minibot::Kinematics::getLastMinibotState();
			state.joint_state.header.stamp = ros::Time::now();
			state.joint_state.header.seq = joint_states_seq++;
			Minibot::pub_joint_state.publish(state.joint_state);

			loop_rate.sleep();
		}

		ros::waitForShutdown();
		ROS_INFO_STREAM("shutting down minibot server node");

	} catch (ros::Exception &e) {
		ROS_ERROR("Error occured: %s ", e.what());
	}
	return 0;

}

