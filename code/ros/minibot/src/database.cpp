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

#include <boost/foreach.hpp>
#include "database.h"
#include "node.h"
#include "utils.h"

namespace Minibot {
namespace Database {

std::string database_prefix = "DATABASE:";

#define LOG_NAME "database"

using namespace mongodb_store;

MessageStoreProxy* messageStore = NULL;

std::string settings_store_db_key = "settings_store";
std::string pose_store_db_key = "pose_store";
std::string priramme_store_db_key = "programme_store";


void init() {
	// initialize the proxy to mongo
	ros::NodeHandle nh;
	messageStore = new MessageStoreProxy (nh);

	// read the settings to ensure that it is initialized
	getSettings();
}

minibot::Configuration getSettings() {

	minibot::Configuration settings;
	std::vector< boost::shared_ptr<minibot::Configuration> > results;
	if (messageStore->queryNamed<minibot::Configuration>(settings_store_db_key, results)) {
		// expect only one result
		if (results.size() != 1) {
			ROS_DEBUG_STREAM_NAMED(LOG_NAME, "Minibot::Database::init" <<
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
	return settings;
}

void setSettings(const minibot::Configuration & settings) {
	ros::NodeHandle nh;
	messageStore->updateNamed(settings_store_db_key, settings);
}

bool handleDatabaseAction(minibot::Database::Request &req,
						  minibot::Database::Response &res) {
	switch (req.type) {
	case minibot::Database::Request::READ_SETTINGS:
		res.configuration = getSettings();
		res.error_code.val = minibot::ErrorCodes::SUCCESS;

		break;
	case minibot::Database::Request::WRITE_SETTINGS:
		setSettings(req.configuration);
		res.error_code.val = minibot::ErrorCodes::SUCCESS;
		break;
	case minibot::Database::Request::READ_POSES:
	case minibot::Database::Request::WRITE_POSES:
	case minibot::Database::Request::READ_PROGRAMME:
	case minibot::Database::Request::WRITE_PROGRAMME:
	default:
		ROS_DEBUG_STREAM_NAMED (LOG_NAME, "Minibot::Database::handleDatabaseAction invalid type=" << req.type);
		pub_msg.publish(Minibot::Utils::createMsg(database_prefix + err_msg_prefix  + "Don't know the database action " + std::to_string(req.type) ));
		res.error_code.val = minibot::ErrorCodes::FAILURE;
		return true;
	}

	return true;
}

}
}
