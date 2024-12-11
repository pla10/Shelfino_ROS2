/*
 * Subscriber.h
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */


/*
 * Subscriber.h
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */
#ifndef SRC_ZMQ_LAYER_SUBSCRIBER_H_
#define SRC_ZMQ_LAYER_SUBSCRIBER_H_

#include "zmq.hpp"
#include <string>
#include <functional>
#include <single_thread.hpp>

namespace ZMQCommon {

class Subscriber : protected Common::single_thread {
  public:
    typedef std::function< void (const char *topic, const char *buf, size_t size, void *data) > callback_t;

    Subscriber();
    // int init(AConnection & conn, std::string topic, std::string address);
    virtual ~Subscriber();
    void register_callback(callback_t callback, void* user_data = nullptr);

    using Common::single_thread::start;
    bool start(std::string address, std::string topic);
    bool stop();
    bool isAlive() {return Common::single_thread::isAlive();}

  private:
    std::unique_ptr<zmq::context_t> context;

    std::string topic_;
    std::string address_;
    size_t topiclen_;

    void worker(const bool& terminating);

    static const int MaxMessageSize = 5000000;
    callback_t recv_callback;
    void * data_;
};

}

#endif /* SRC_ZMQ_LAYER_SUBSCRIBER_H_ */