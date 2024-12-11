/*
 * RequesterSimple.h
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */
#ifndef SRC_ZMQ_LAYER_RequesterSimple_H_
#define SRC_ZMQ_LAYER_RequesterSimple_H_

#include "zmq.hpp"
#include <string>
#include <functional>
#include <condition_variable>
#include <queue>
#include <mutex>
#include <map>
#include <single_thread.hpp>

namespace ZMQCommon {

class RequesterSimple {
  public:
        typedef enum {
      STATUS_OK,
      STATUS_TIMEOUT,
      STATUS_ERROR
    } status_t;

    typedef struct _req_ {
      std::string rcvd;
      status_t status;

      _req_(): rcvd(""), status(STATUS_OK) {}
    } request_t;

    RequesterSimple(std::string address);
    virtual ~RequesterSimple();

    
    bool request(const std::string& in, std::string& out, status_t& status);
    std::string statusToStr(status_t status);
    bool close();

  private:

    uint32_t request_uid;

    uint32_t timeout_ms = 1000;
    uint32_t retries = 3;
    std::unique_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> socket_;

    std::string address_;
    std::mutex mutex_request;

    bool exception_fired;

    zmq_pollitem_t item;
    
    bool resetSocket(zmq_pollitem_t& pollitem);
};

}

#endif /* SRC_ZMQ_LAYER_RequesterSimple_SAFE_H_ */
