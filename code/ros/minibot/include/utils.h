#ifndef MININBOT_UTILS_H_
#define MININBOT_UTILS_H_

#include <map>

enum MinibotError
{
  OK = 1,
  IK_NOT_AVAILABLE = -1,
  UKNOWN = -99
};

void initErrorMap();
extern std::map<int, std::string> minibotErrorMap;


#endif
