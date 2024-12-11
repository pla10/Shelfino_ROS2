/*
 * RequesterSimple.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */

#include <RequesterSimple.hpp>
#include <iostream>
#include <stdexcept>
#include <zhelpers.hpp>

using namespace std;
namespace ZMQCommon {

#if ZMQ_VERSION_MAJOR >= 3
// ZMQ version >= 3 timeout in milliseconds
#define ZMQ_TOUT_MSEC 1
#else
// ZMQ version < 3 timeout in microseconds
#define ZMQ_TOUT_MSEC 1000
#endif

RequesterSimple::RequesterSimple(std::string address){
  context = unique_ptr<zmq::context_t>(new zmq::context_t());
  request_uid = 0;
  address_ = address;
  //retries = 3;
  exception_fired = false;
  resetSocket(item);
}

bool RequesterSimple::close() {

  return zmq_term(context.get());;
}



bool RequesterSimple::request(const std::string& in, std::string& out, status_t& status) {
  std::unique_lock<std::mutex> lock(mutex_request);

  request_uid++; // let request uid start from 1
  string recvd_str = "";
  request_t request_response;
// send request
  try{
    if (!s_send(*socket_, in)) {
      cerr << "RequesterSimple::worker(): error sending string to socket." << endl;
      // wait before retrial
      this_thread::sleep_for(chrono::milliseconds(60000));
      return false;
    }
    // wait for response
    // wait for reply

    bool datain = false;
    int local_retries = retries;
    while (local_retries) {
      local_retries--;
      int rc = zmq_poll(&item, 1, timeout_ms * ZMQ_TOUT_MSEC); // convert to msec

      switch (rc) {
        // timeout, no messages
        case 0:
          continue;
          // terminated, exit process
        case ETERM:
            return false;
           continue;
        default:
          if (rc < 0) {
            cerr << "RequesterSimple::worker(), zmq_poll exit code " << rc
                << ". Ignoring." << endl;
            continue;
          }
      }

      // received something, exit loop!
      datain = true;
      break;
    }

    if (!datain) {
      // no data received after timeout
      cerr << "RequesterSimple::worker(): TIMEOUT! (" << timeout_ms
          << " ms / " << retries << " retries)" << endl;
      cerr.flush();
      request_response.rcvd = "";
      request_response.status = STATUS_TIMEOUT;
      // reset socket to avoid EFSM exception (REQ-REP mechanism allows only one request at a time)
      resetSocket(item);
    } else {
      // data ready to be notified
      recvd_str = s_recv(*socket_);
      request_response.rcvd = recvd_str;
      request_response.status = STATUS_OK;
      out = recvd_str;
      /*if (recv_callback) {
        // notify reception of new string message
        recv_callback(recvd_str, callback_user_data);
      }*/
    }
  } catch (const zmq::error_t& ex) {
    switch (ex.num()) {
      case ETERM:
        break;
      case EFSM: /*Operation cannot be accomplished in current state*/
        resetSocket(item);
        this_thread::sleep_for(chrono::milliseconds(250));
        break;
      default:
        cerr << "RequesterSimple::worker(): zmq exception ("<< ex.num() << ") \"" << ex.what()
                    << "\"" << endl;
    }
    exception_fired = true;
  } catch (const exception& e) {
    cerr << "RequesterSimple::worker(): exception \"" << e.what() << "\""
        << endl;
    exception_fired = true;
  }

  status = request_response.status;
    return true;
}

bool RequesterSimple::resetSocket(zmq_pollitem_t& pollitem) {
  // create socket
  socket_.reset(new zmq::socket_t(*context, ZMQ_REQ));

  try {
    // set linger value to zero
    int linger = 0;
    socket_->setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    socket_->connect(address_.c_str());
  } catch (const exception& e) {
    cerr << "Exception connecting to \"" << address_ << "\", \"" << e.what() << "\"" << endl;
    return false;
  }

  // poll item
  pollitem.socket = (void*)*socket_;
  pollitem.events = ZMQ_POLLIN;

  return true;
}


RequesterSimple::~RequesterSimple() {
 
}
/*
void RequesterSimple::register_callback(callback_t callback, void* data) {
  recv_callback = callback;
  callback_user_data = data;
}*/

string RequesterSimple::statusToStr(status_t status) {
  switch (status) {
    case STATUS_OK:
      return "STATUS_OK";
    case STATUS_TIMEOUT:
      return "STATUS_TIMEOUT";
    case STATUS_ERROR:
      return "STATUS_ERROR";
    default:
      throw runtime_error("RequesterSimple::statusToStr(), unknown status (" + to_string(status) + ")");
  }
}

}
