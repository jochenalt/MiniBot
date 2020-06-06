#include "planner.h"

namespace Minibot {

#define LOG_NAME "planner"


namespace Planner {
void init() {

  ROS_DEBUG_STREAM_NAMED(LOG_NAME, "planner Debug stream");
  ROS_INFO_STREAM_NAMED(LOG_NAME, "planner Info stream");
  ROS_DEBUG_STREAM("planner debug");
  ROS_INFO_STREAM( " planner info");

}

}

}
