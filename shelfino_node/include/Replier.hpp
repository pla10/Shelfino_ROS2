/*
 * Replier.h
 *
 *  Created on: Jun 25, 2015
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */
#ifndef ZMQ_LAYER_REPLIER_H_
#define ZMQ_LAYER_REPLIER_H_

#include <string>
#include <functional>
#include "zmq.hpp"
#include <single_thread.hpp>

namespace Common{

class Replier : single_thread {

  public:
    typedef std::function< void (const std::string& received, void *data, std::string& tosend) > callback_t;

    Replier();

    using single_thread::start;
    bool start(std::string address);
    bool stop();
    void register_callback(callback_t callback, void* user_data);

    virtual ~Replier();

  private:

    std::unique_ptr<zmq::context_t> context;
    uint32_t timeout_ms;
    std::string address;
    std::unique_ptr<std::thread> innerThread;
    callback_t recv_callback;
    void * callback_user_data;

    void worker(const bool& terminating);

};


}
#endif /* ZMQ_LAYER_REPLIER_H_ */
