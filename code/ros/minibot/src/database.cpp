/*
 * database.cpp
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */


#include "mongodb_store/message_store.h"
#include "geometry_msgs/Pose.h"
#include <minibot/Configuration.h>

#include <boost/foreach.hpp>
#include "database.h"

namespace Minibot {
namespace Database {

#define LOG_NAME "database"

using namespace mongodb_store;

MessageStoreProxy* messageStore = NULL;

std::string settings_store_db_key = "settings_store";
std::string pose_store_db_key = "pose_store";
std::string priramme_store_db_key = "programme_store";

minibot::Configuration settings;


void init(const ros::NodeHandle& nh) {
    //Create object which does the work for us.
	messageStore = new MessageStoreProxy(nh);

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

	/*
	 global statements, robotstates, configuration
	  db = MessageStoreProxy()
	  (poseStorage, meta) = db.query_named("default_pose_storage1",PoseStorage._type)
	  if poseStorage is None:
	    poseStorage = PoseStorage()
	    db.insert_named("default_pose_storage1",poseStorage)
	  robotstates = poseStorage.states

	  (programme, meta) = db.query_named("default_programme1", Programme._type)
	  if programme is None:
	    programme = Programme()
	    db.insert_named("default_programme1",programme)
	  statements = programme.statements

	  (configuration, meta) = db.query_named("mysettings", Configuration._type)
	  if configuration is None:
	    configuration = Configuration()
	    configuration.theme = "cyborgTheme"
	    configuration.angle_unit = Configuration.ANGLE_UNIT_RAD
	    configuration.save_after
	    configuration.save_after.secs = 3000
	    configuration.save_after.nsecs = 0

	    db.insert_named("mysettings",configuration)
*/

}
}
}
