/**
 * This is a strange class I stole from the github source of mongodb_store, it is declared in
 * "mongodb_store/message_store.h" but not implemented in any library I found,
 * so its implementation is needed here
 */

#include "mongodb_store/message_store.h"

namespace mongodb_store {

const mongo::BSONObj MessageStoreProxy::EMPTY_BSON_OBJ = mongo::BSONObj();

mongodb_store_msgs::StringPair makePair(const std::string &_first, const std::string &_second) {
	mongodb_store_msgs::StringPair pair;
	pair.first = _first;
	pair.second = _second;
	return pair;
}
}
