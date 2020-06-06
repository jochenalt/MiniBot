#include "utils.h"

std::map<int, std::string> minibotErrorMap;

void initErrorMap() {
    minibotErrorMap = {
	{ MinibotError::OK, "OK" },
	{ MinibotError::IK_NOT_AVAILABLE, "One or more ik solutions could not be found" },
	{ MinibotError::UKNOWN, "unknown error" }
    };
}
