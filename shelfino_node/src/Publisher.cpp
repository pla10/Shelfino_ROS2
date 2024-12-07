/*
 * Publisher.c
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */

#include <Publisher.hpp>

#include <iostream>

using namespace std;

namespace Common {

Publisher::Publisher(string address): context_(1), socket_(context_,ZMQ_PUB) {
	socket_.bind(address.c_str());
  int linger = 0;
  socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
}

bool Publisher::send(const char* topic, const char * buf, size_t message_size) {
	int res;
	// send the message, add the topic to the message
	// std::cerr  << "[layer send] topic "<< topic_ << "\t"  << topic_.length() << "\t" << toffset_<< std::endl;
	size_t off = strlen(topic);

	zmq::message_t msg(message_size + off);
	memcpy(msg.data(), topic, off);
	memcpy(static_cast<char *>(msg.data()) + off, buf, message_size);
	// std::cerr <<"[layer send] " << msg.size() << "\t" << topic_ << "\t"<< (char*)msg.data() << "\t" << buf << std::endl;
	res = socket_.send(msg);
	if (res < 0) {
		std::cerr << "Publisher::send(), error: " << res << std::endl;
		return false;
	}

	return true;
}

Publisher::~Publisher() {
  //zmq_close(*socket_);
  //zmq_term(context_.get());
}

}
