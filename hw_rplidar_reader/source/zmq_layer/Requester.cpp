/*
 * Requester.cpp
 *
 *  Created on: Oct 9, 2014
 *      Author: "Alessio Colombo <colombo@disi.unitn.it>"
 */

#include <Requester.hpp>
#include <iostream>
#include <stdexcept>
#include <zhelpers.hpp>

using namespace std;
namespace Common {

#if ZMQ_VERSION_MAJOR >= 3
// ZMQ version >= 3 timeout in milliseconds
#define ZMQ_TOUT_MSEC 1
#else
// ZMQ version < 3 timeout in microseconds
#define ZMQ_TOUT_MSEC 1000
#endif

Requester::Requester():
    single_thread(thread_fun_t([this](const bool& t){this->worker(t);}), "Requester"),
    request_uid(0), address_("") {
  context = unique_ptr<zmq::context_t>(new zmq::context_t());
}

bool Requester::Test() {
  Requester req;

  req.setMaxRetries(1);
  req.setTimeout(250);
  req.start("tcp://localhost:8888");

  string request = "prova", output = "";
  uint16_t runs = 2;
  status_t status;
  while (--runs) {
    cout << "Sending \"" << request << "\"..." << endl;
    req.request("ciao",output,status);
    cout << "Received \"" << output << "\" with status \"" << statusToStr(status) << "\"" << endl;
    cout.flush();
    cerr.flush();
    this_thread::sleep_for(chrono::milliseconds(500));
  }
  req.stop();

  req.start("tcp://localhost:5569");
  req.setMaxRetries(2);
  req.setTimeout(1000);
  runs = 5;
  request = "ciaociao";
  while (--runs) {
    cout << "Sending \"" << request << "\"..." << endl;
    req.request("ciao",output,status);
    cout << "Received \"" << output << "\" with status \"" << statusToStr(status) << "\"" << endl;
    cout.flush();
    cerr.flush();
    this_thread::sleep_for(chrono::milliseconds(500));
  }
  req.stop();

  return true;
}

bool Requester::start(string address) {
  if (single_thread::isAlive()) {
    return true;
  }

  // clear data structures
  std::queue<queue_elem_t> empty;
  std::swap( request_queue, empty );
  uid_request_map.clear();

  request_uid = 0;
  address_ = address;

  return single_thread::start();
}

bool Requester::stop() {
  if (!single_thread::isAlive()) {
    return true;
  }

  zmq_term(context.get());

  cond_request.notify_all();
  bool r = single_thread::stop();
  cond_event.notify_all();

  return r;
}

bool Requester::request(const std::string& in, std::string& out, status_t& status) {
  status = STATUS_ERROR;
  if (!single_thread::isAlive()) {
    return false;
  }

  queue_elem_t qe;
  mutex_queue.lock();
  qe.s = in;
  request_uid++; // let request uid start from 1
  qe.uid = request_uid;
  request_queue.push(qe);
  mutex_queue.unlock();

  cond_request.notify_all();

  // wait for response
  while(single_thread::isAlive()) {
    unique_lock<mutex> lock(mutex_event);
    cond_event.wait(lock);
    lock_guard<mutex> guard(mutex_map);
    uid_request_map_t::iterator it = uid_request_map.find(qe.uid);
    if (it != uid_request_map.end()) {
      out = (*it).second.rcvd;
      status = (*it).second.status;
      break;
    }
  }

  if (!single_thread::isAlive()) {
    return false;
  } else {
    return (status == STATUS_OK)?true:false;
  }
}

bool Requester::resetSocket(zmq_pollitem_t& pollitem) {
  // create socket
  socket_.reset(new zmq::socket_t(*context, ZMQ_REQ));

  try {
    // set linger value to zero
    int linger = 0;
    socket_->setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
    socket_->connect(address_.c_str());
  } catch (const exception& e) {
    cerr << "Exception connecting to \"" << address_ << "\", \"" << e.what() << "\"" << endl;
    single_thread::notifyTermination();
    cond_event.notify_all();
    return false;
  }

  // poll items
  pollitem.socket = (void*)*socket_;
  pollitem.events = ZMQ_POLLIN;

  return true;
}

void Requester::worker(const bool& terminating) {
  zmq_pollitem_t items[1];
  int rc = 0;
  bool datain = false, request_popped = false, exception_fired = false;
  string recvd_str = "";
  queue_elem_t current_request;
  request_t request_response;
  uint32_t local_retries = 0;

  if (!resetSocket(items[0])) {
    return;
  }

  cout << "Requester::worker(), requesting to " << address_ << "" << endl;
  //cout.flush();

  while (!terminating) {
    try {
      current_request.uid = 0;
      current_request.s = "";
      request_popped = false;

      // wait until new data come across
      while (request_queue.empty() && !terminating) {
        unique_lock<mutex> lock(mutex_request);
        cond_request.wait(lock);
      }
      if (terminating) break;

      // get first request in the queue
      mutex_queue.lock();
      current_request = request_queue.front();
      mutex_queue.unlock();

      // send request
      if (!s_send(*socket_, current_request.s)) {
        cerr << "Requester::worker(): error sending string to socket." << endl;
        // wait before retrial
        this_thread::sleep_for(chrono::milliseconds(500));
        continue;
      }

      // we successfully sent the data, remove from queue
      mutex_queue.lock();
      request_queue.pop();
      mutex_queue.unlock();
      request_popped = true;

      // wait for reply
      datain = false;
      local_retries = retries;
      while (local_retries && !terminating) {
        local_retries--;
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
              cerr << "Requester::worker(), zmq_poll exit code " << rc
                  << ". Ignoring." << endl;
              continue;
            }
        }

        // received something, exit loop!
        datain = true;
        break;
      }

      if (terminating) break;

      if (!datain) {
        // no data received after timeout
        cerr << "Requester::worker(): TIMEOUT! (" << timeout_ms
            << " ms / " << retries << " retries)" << endl;
        cerr.flush();
        request_response.rcvd = "";
        request_response.status = STATUS_TIMEOUT;
        // reset socket to avoid EFSM exception (REQ-REP mechanism allows only one request at a time)
        resetSocket(items[0]);
      } else {
        // data ready to be notified
        recvd_str = s_recv(*socket_);
        request_response.rcvd = recvd_str;
        request_response.status = STATUS_OK;
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
          resetSocket(items[0]);
          this_thread::sleep_for(chrono::milliseconds(250));
          break;
        default:
          cerr << "Requester::worker(): zmq exception ("<< ex.num() << ") \"" << ex.what()
                      << "\"" << endl;
      }
      exception_fired = true;
    } catch (const exception& e) {
      cerr << "Requester::worker(): exception \"" << e.what() << "\""
          << endl;
      exception_fired = true;
    }

    // check if we got interrupted while serving a request..
    if (exception_fired && !terminating && request_popped) {
      // in this case we got an exception while we were serving a request
      request_response.rcvd = "";
      request_response.status = STATUS_ERROR;
    }

    if (request_popped) {
      // save response to map
      lock_guard<mutex> guard(mutex_map);
      uid_request_map[current_request.uid] = request_response;
      cond_event.notify_all();
    }
  }

  //cout << "Requester::worker(), goodbye :(" << endl;
  //cout.flush();
}

Requester::~Requester() {
  stop();
}
/*
void Requester::register_callback(callback_t callback, void* data) {
  recv_callback = callback;
  callback_user_data = data;
}*/

string Requester::statusToStr(status_t status) {
  switch (status) {
    case STATUS_OK:
      return "STATUS_OK";
    case STATUS_TIMEOUT:
      return "STATUS_TIMEOUT";
    case STATUS_ERROR:
      return "STATUS_ERROR";
    default:
      throw runtime_error("Requester::statusToStr(), unknown status (" + to_string(status) + ")");
  }
}

}
