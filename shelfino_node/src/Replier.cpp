/*
 * Replier.cpp
 *
 *  Created on: Jun 25, 2015
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */
#include <Replier.hpp>
#include <zhelpers.hpp>
#include <system_error>

using namespace std;

namespace Common {

#if ZMQ_VERSION_MAJOR >= 3
// ZMQ version >= 3 timeout in milliseconds
#define ZMQ_TOUT_MSEC 1
#else
// ZMQ version < 3 timeout in microseconds
#define ZMQ_TOUT_MSEC 1000
#endif

Replier::Replier() :
    single_thread(thread_fun_t([this](const bool& t){this->worker(t);}), "Replier"),
    timeout_ms(1000), callback_user_data(nullptr) {
  context = unique_ptr<zmq::context_t>(new zmq::context_t());
}

Replier::~Replier() {
  stop();
}

bool Replier::start(string address) {
  if (single_thread::isAlive()) {
    return true;
  }

  this->address = address;
  return single_thread::start();
}

bool Replier::stop() {
  if (!single_thread::isAlive()) {
    return true;
  }

  zmq_term(context.get());

  return single_thread::stop();
}

void Replier::worker(const bool& terminating) {
  zmq_pollitem_t items[1];
  zmq::message_t message;
  std::string recvd_str, tosend;
  uint16_t max_retrials = 10, retrial = 0;
  int32_t rc = 0;
  uint32_t send_interval_ms = 100; // ms
  bool success = false;

  try {
    zmq::socket_t socket_(*context, ZMQ_REP);
    socket_.bind(address.c_str());
    int linger = 0;
    socket_.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

    // poll items
    items[0].socket = (void*)socket_;
    items[0].events = ZMQ_POLLIN;

    while (!terminating) {
      rc = zmq_poll(&items[0], 1, timeout_ms * ZMQ_TOUT_MSEC); // convert to msec

      switch (rc) {
        // timeout, no messages
        case 0:
          continue;
          // terminated, exit process
        case ETERM:
          single_thread::notifyTermination();
          continue;
        default:
          if (rc < 0) {
            cerr << "Requester_safe::worker(), zmq_poll exit code " << rc
                << ". Ignoring." << endl;
            continue;
          }
      }

      if (terminating) {
        break;
      }

      recvd_str = s_recv(socket_);

      if (recv_callback) {
        recv_callback(recvd_str, callback_user_data, tosend);
        retrial = max_retrials;
        success = false;
        while (!terminating && ((retrial--) + 1)) {
          if (s_send(socket_, tosend)) {
            success = true;
            break;
          }
          cerr << "Replier::worker(), error sending reply!" << endl;
          cerr.flush();
          this_thread::sleep_for(chrono::milliseconds(send_interval_ms));
        }
        if (!terminating && !success) {
          cerr << "Replier::worker(), unable to send reply after " << max_retrials << " trials. Skipping.." << endl;
          cerr.flush();
        }

      } else {
        cerr << "Replier::worker(), received request but no callback set!" << endl;
        cerr.flush();
      }
    }

  } catch (exception& e) {
    cerr << "Replier::worker() exception generated, exiting." << endl;
    cerr << "\"" << e.what() << "\"" << endl;
    cerr.flush();
  }
}

void Replier::register_callback(callback_t callback, void* user_data) {
  recv_callback = callback;
  callback_user_data = user_data;
}

}
