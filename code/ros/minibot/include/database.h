/*
 * database.h
 *
 *  Created on: Jun 10, 2020
 *      Author: jochenalt
 */

#ifndef INCLUDE_DATABASE_H_
#define INCLUDE_DATABASE_H_

#include "ros/ros.h"
#include <minibot/Configuration.h>
#include <minibot/DatabaseAction.h>
#include <minibot/GlobalPlan.h>

namespace Minibot {

extern minibot::Configuration settings;
extern minibot::PoseStorage pose_store;
extern minibot::Programme programme_store;

namespace Database {


	// call me before anything happens in the database
void construct();

	// read/write settings from the database
void readSettings();
void setSettings(const minibot::Configuration & settings);

	// read/write pose storarge
void setPoseStorage(const minibot::PoseStorage & settings);
void readPoseStorage();

	// read/write programme
void setProgramme(const minibot::Programme & settings);
void readProgramme();

	// handle incoming database Requests
bool handleDatabaseAction(minibot::DatabaseAction::Request &req,
						  minibot::DatabaseAction::Response &res);

}
}


#endif /* INCLUDE_DATABASE_H_ */
