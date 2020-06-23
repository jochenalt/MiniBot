/*
 * database.cpp
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#include <stdio.h>
#include <stdlib.h>

#include "mongodb_store/message_store.h"

#include <minibot/Configuration.h>
#include <minibot/ErrorCodes.h>

#include "database.h"
#include "node.h"
#include "utils.h"
#include "planner.h"

namespace Minibot {

minibot::Configuration settings;
minibot::PoseStorage pose_store;
minibot::Programme programme_store;


namespace Database {

std::string database_prefix = "DATABASE:";

#define LOG_NAME "database"

using namespace mongodb_store;

MessageStoreProxy* messageStore = NULL;

// names of main objects in mongo
std::string settings_store_db_key 	= "settings_store";
std::string pose_store_db_key		= "pose_store";
std::string programme_store_db_key 	= "programme_store";


void construct() {
	ROS_INFO_STREAM_NAMED(LOG_NAME, "module database init");

	// initialize the proxy to mongodb
	ros::NodeHandle nh;
	messageStore = new MessageStoreProxy (nh);

	// read all main objects to ensure that they are initialized
	readSettings();
	readPoseStorage();
	readProgramme();
}

void readSettings() {
	std::vector< boost::shared_ptr<minibot::Configuration> > results;
	if (messageStore->queryNamed<minibot::Configuration>(settings_store_db_key, results)) {
		// expect only one result
		if (results.size() != 1) {
			ROS_DEBUG_STREAM_NAMED(LOG_NAME, "Minibot::Database::getSettings" <<
					" more than one settings object found");
		} else {
			settings = *results[0];
		}
	} else {
		settings.theme = "cyborg";
		settings.angle_unit = minibot::Configuration::ANGLE_UNIT_RAD;
		settings.save_after = ros::Duration(3000,0);
		messageStore->insertNamed(settings_store_db_key, settings);
	}
}


void setSettings(const minibot::Configuration & settings) {
	messageStore->updateNamed(settings_store_db_key, settings);
	Minibot::settings = settings;
}

void setPoseStorage(const minibot::PoseStorage & pose_store) {
	messageStore->updateNamed(posestore_prefix, pose_store);
	Minibot::pose_store = pose_store;
}


void  readPoseStorage() {
	minibot::PoseStorage tmp_pose_store;
	std::vector< boost::shared_ptr<minibot::PoseStorage> > results;
	if (messageStore->queryNamed<minibot::PoseStorage>(posestore_prefix, results)) {
		// expect only one result
		if (results.size() != 1) {
			ROS_DEBUG_STREAM_NAMED(LOG_NAME, "Minibot::Database::getPoseStorage" <<
					" more than one pose store object found");
		} else {
			tmp_pose_store = *results[0];
		}
	} else {
		messageStore->insertNamed(posestore_prefix, tmp_pose_store);
	}
	Minibot::pose_store = tmp_pose_store;
}


void readProgramme() {
	minibot::Programme tmp_programme;
	std::vector< boost::shared_ptr<minibot::Programme> > results;
	if (messageStore->queryNamed<minibot::Programme>(programme_store_db_key, results)) {
		// expect only one result
		if (results.size() != 1) {
			ROS_DEBUG_STREAM_NAMED(LOG_NAME, "Minibot::Database::getProgramme" <<
					" more than one programme store object found");
		} else {
			tmp_programme = *results[0];
		}
	} else {
		messageStore->insertNamed(programme_store_db_key, tmp_programme);
	}
	Minibot::programme_store = tmp_programme;
}


void setProgramme(const minibot::Programme & programme) {
	messageStore->updateNamed(programme_store_db_key, programme);
	Minibot::programme_store = programme;

}

bool handleDatabaseAction(minibot::DatabaseAction::Request &req,
						  minibot::DatabaseAction::Response &res) {
	switch (req.type) {
	case minibot::DatabaseAction::Request::READ_SETTINGS:
		res.configuration = Minibot::settings;
		res.error_code.val = minibot::ErrorCodes::SUCCESS;
		break;
	case minibot::DatabaseAction::Request::WRITE_SETTINGS:
		setSettings(req.configuration);
		res.error_code.val = minibot::ErrorCodes::SUCCESS;
		break;
	case minibot::DatabaseAction::Request::READ_POSES:
		res.pose_store = Minibot::pose_store;
		res.error_code.val = minibot::ErrorCodes::SUCCESS;
		break;
	case minibot::DatabaseAction::Request::WRITE_POSES:
		setPoseStorage(req.pose_store);
		res.error_code.val = minibot::ErrorCodes::SUCCESS;
		break;
	case minibot::DatabaseAction::Request::READ_PROGRAMME:
		res.programme_store = Minibot::programme_store;
		res.error_code.val = minibot::ErrorCodes::SUCCESS;
		break;
	case minibot::DatabaseAction::Request::WRITE_PROGRAMME:
		setProgramme(req.programme_store);

		// initiate planning
		Minibot::Planner::plan();

		// return the programme in case we changed anything
		res.programme_store = Minibot::programme_store;
		res.error_code.val = minibot::ErrorCodes::SUCCESS;
		break;
	default:
		ROS_ERROR_STREAM_NAMED (LOG_NAME, "Minibot::Database::handleDatabaseAction invalid type=" << req.type);
		pub_msg.publish(Minibot::Utils::createMsg(database_prefix + err_msg_prefix  + "Don't know the database action " + std::to_string(req.type) ));
		res.error_code.val = minibot::ErrorCodes::FAILURE;
		return true;
	}

	return true;
}

}
}
