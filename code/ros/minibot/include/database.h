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
#include <minibot/Database.h>

namespace Minibot {
namespace Database {

extern minibot::Configuration settings;

	// call me before anything happens in the database
void init();

	// read/write settings from the database
minibot::Configuration getSettings();
void setSettings(const minibot::Configuration & settings);

	// read/write pose storarge
void setPoseStorage(const minibot::PoseStorage & settings);
minibot::PoseStorage getPoseStorage();

	// read/write programme
void setProgramme(const minibot::Programme & settings);
minibot::Programme getProgramme();

	// handle incoming database Requests
bool handleDatabaseAction(minibot::Database::Request &req,
						  minibot::Database::Response &res);

}
}


#endif /* INCLUDE_DATABASE_H_ */
