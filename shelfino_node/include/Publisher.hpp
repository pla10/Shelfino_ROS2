/*
 * Publisher.h
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */
#ifndef SRC_ZMQ_LAYER_PUBLISHER_H_
#define SRC_ZMQ_LAYER_PUBLISHER_H_

#include "zmq.hpp"
#include <string>
#include <memory>

namespace Common {

class Publisher {
public:
	Publisher(std::string address);
	// int init(BaseAConnection & conn, std::string topic, std::string address);
	bool send(const char* topic, const char * buf, size_t message_size);
	virtual ~Publisher();

private:
	//std::string address_;
	zmq::context_t context_;
	zmq::socket_t socket_;
};

}

#endif /* SRC_ZMQ_LAYER_PUBLISHER_H_ */
