/*
 * Requester.h
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */
#ifndef SRC_ZMQ_LAYER_REQUESTER_H_
#define SRC_ZMQ_LAYER_REQUESTER_H_

#include "zmq.hpp"
#include <string>
#include <functional>
#include <condition_variable>
#include <queue>
#include <mutex>
#include <map>
#include <single_thread.hpp>

namespace Common {

class Requester : single_thread {
  public:
    //typedef std::function< void (const std::string& received, void *data) > callback_t;

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

    Requester();
    virtual ~Requester();

    //void register_callback(callback_t callback, void*);
    using single_thread::start;
    bool start(std::string address);

    bool stop();
    bool request(const std::string& in, std::string& out, status_t& status);
    void setTimeout(uint32_t timeout_ms) {this->timeout_ms = timeout_ms;}
    void setMaxRetries(uint32_t retries) {this->retries = retries;}
    static std::string statusToStr(status_t status);

    static bool Test();

  private:

    typedef struct _qe_ {
      std::string s;
      uint32_t uid;

      _qe_(): s(""), uid(0) {}
    } queue_elem_t;

    typedef std::map<uint32_t, request_t> uid_request_map_t;

    std::unique_ptr<zmq::context_t> context;
    std::unique_ptr<zmq::socket_t> socket_;

    uint32_t request_uid;

    uint32_t timeout_ms = 2000;
    uint32_t retries = 5;

    std::string address_;

    uid_request_map_t uid_request_map;

    std::condition_variable cond_request, cond_event;

    std::queue<queue_elem_t> request_queue;
    std::mutex mutex_request, mutex_event, mutex_queue, mutex_map;

    void worker(const bool& terminating);
    bool resetSocket(zmq_pollitem_t& pollitem);
};

}

#endif /* SRC_ZMQ_LAYER_REQUESTER_SAFE_H_ */
