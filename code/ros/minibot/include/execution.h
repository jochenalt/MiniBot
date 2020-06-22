/*
 * execution.h
 *
 *  Created on: Jun 22, 2020
 *      Author: jochen
 */

#ifndef INCLUDE_EXECUTION_H_
#define INCLUDE_EXECUTION_H_


#include "ros/ros.h"


#include <minibot/GlobalPlan.h>


namespace Minibot {
namespace Execution {

void init();

// compile a programme and create a global plan consisting of local plans
minibot::ErrorCodes  execute (const minibot::LocalPlan& local_plan);


}
}


#endif /* INCLUDE_EXECUTION_H_ */
