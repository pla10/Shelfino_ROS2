/*
 * single_thread.h
 *
 *  Created on: July 6, 2015
 *      Author: "Alessio Colombo <alessio.col@gmail.com>"
        Reviewed by "Fabiano Zenatti <fabiano.zenatti@gmail.com>""
 */

#ifndef SINGLE_THREAD_H_
#define SINGLE_THREAD_H_

#include <condition_variable>
#include <thread>
#include <string>
#include <functional>
#include <system_error>
#include <iostream>
#include <stdexcept>

namespace Common {

class single_thread {
  public:

    typedef std::function<void(const bool&)> thread_fun_t;
    typedef std::function<void()> terminating_fun_t;

    single_thread(const std::string& thread_name = "single_thread") :
        alive(false), terminating(false), joined(true),
        exception_thrown(false), default_tout_ms(60000),
        thread_name(thread_name) {
      custom_worker = [](const bool&){};
      custom_terminating_callback = [](){};

    }

    single_thread(thread_fun_t f, const std::string& thread_name) :
        alive(false), terminating(false), joined(true),
        exception_thrown(false), default_tout_ms(60000),
        thread_name(thread_name), custom_worker(f) {
          custom_terminating_callback = [](){};
    }

    single_thread(thread_fun_t f, terminating_fun_t ft, const std::string& thread_name) :
        alive(false), terminating(false), joined(true),
        exception_thrown(false), default_tout_ms(60000),
        thread_name(thread_name), custom_worker(f) {
          custom_terminating_callback = ft;
    }

    virtual bool start() {
      std::lock_guard<std::mutex> lock(mutex_run);
      if (alive) {
        return true;
      }

      join_if_needed();

      exception_thrown = false;
      terminating = false;
      innerThread.reset(new std::thread(&single_thread::worker, this));

      try {
        std::unique_lock<std::mutex> lock_wk(mutex_worker);
        if (cond_worker_start.wait_for(lock_wk, std::chrono::milliseconds(default_tout_ms), [this](){return this->alive;})) {
          //std::cout << thread_name << "::start(), thread started successfully." << std::endl;
          return true;
        } else {
          std::cerr << thread_name << "::start(), thread not started after timeout (" << default_tout_ms << " ms)" << std::endl;
          return false;
        }
      } catch (const std::exception& e) {
        std::cerr << thread_name << "::start(), exception generated: \""<< e.what() << "\"" << std::endl;
        return false;
      }
    }

    virtual bool stop() {
      std::lock_guard<std::mutex> lock(mutex_run);
      if (!alive && joined) {
        return true;
      }

      if (join_if_needed()) {
        return true;
      }

      mutex_stop_req.lock(); //// MODIFICATO IN DATA 14-LUG-2017
      terminating = true;
      mutex_stop_req.unlock(); ///// MODIFICATO IN DATA 14-LUG-2017
      cond_worker_stop_req.notify_all();
      bool has_error = false;
      //std::cout << thread_name << "::stop(), stopping thread (" << default_tout_ms << " ms timeout)" << std::endl;
      //std::cout.flush();
      try {
        std::unique_lock<std::mutex> lock_wk(mutex_worker);
        if (cond_worker_stop.wait_for(lock_wk, std::chrono::milliseconds(default_tout_ms), [this](){return !this->alive;})) {
          //std::cout << thread_name << "::stop(), thread stopped successfully." << std::endl;
        } else {
          std::cerr << thread_name << "::stop(), thread not stopped after timeout" << std::endl;
          has_error = true;
        }
      } catch (const std::exception& e) {
        std::cerr << thread_name << "::stop(), exception generated: \""<< e.what() << "\"" << std::endl;
        has_error = true;
      }

      if (!has_error && innerThread) {
        try {
          innerThread->join();
        } catch (const std::system_error& syserr) {
          std::cerr << thread_name << "::stop(), exception generated while joining thread: \""<< syserr.what() << "\"."
           << std::endl << "No panic! We are strong enough to continue anyway.."<< std::endl;
        }
      }

      std::cout.flush();
      std::cerr.flush();
      return !has_error;
    }

    void notifyTermination() {
      terminating = true;
    }

    bool isAlive() const {
      return alive;
    }
    bool isTerminating() const {
      return terminating;
    }
    bool isExceptionThrown() const {
      return exception_thrown;
    }
    uint32_t getDefaultTimeoutMilli() const {
      return default_tout_ms;
    }
    void setDefaultTimeoutMilli(uint32_t tout_ms) {
      default_tout_ms = tout_ms;
    }

    void setThreadFunction(thread_fun_t tf) {custom_worker = tf;}

    const std::exception& getLastException() const {return last_exception;}

    std::string getThreadName(){ return thread_name; }

    virtual ~single_thread() {stop();}

  protected:

    // function to be called from within the custom_worker() function and
    // will block until stop() is called.
    // this has been done in order to avoid the ugly
    //
    // while(!terminating) {sleep(some time);}
    //
    bool workerSelfWaitForTerminationRequest() {
      if (!alive) {
        return true;
      }

      try {
        std::unique_lock<std::mutex> lock_wk(mutex_stop_req);
        cond_worker_stop_req.wait(lock_wk, [this]{return terminating;});
      } catch (const std::exception& e) {
        std::cerr << thread_name << "::waitForTerminationRequest(), exception " << e.what() << std::endl;
        std::cerr.flush();
        return false;
      }
      return true;
    }

  private:

    std::condition_variable cond_worker_start, cond_worker_stop, cond_worker_stop_req;
    std::mutex mutex_run, mutex_worker, mutex_stop_req;
    bool alive, terminating, joined;
    std::unique_ptr<std::thread> innerThread;
    bool exception_thrown;
    std::exception last_exception;
    uint32_t default_tout_ms;
    std::string thread_name;
    thread_fun_t custom_worker;
    terminating_fun_t custom_terminating_callback;

    bool join_if_needed() {
      if (!alive && !joined) {
        try {
          if (innerThread->joinable()) {
            innerThread->join();
          }
        } catch (const std::system_error& syserr) {
          std::cerr << thread_name << "::stop(), exception generated while joining thread: \""<< syserr.what() << "\"."
           << std::endl << "No panic! We are strong enough to continue anyway.."<< std::endl;
        }
        joined = true;
        return true;
      } else {
        return false;
      }
    }

    void worker() {
      alive = true;
      joined = false;
      cond_worker_start.notify_all();
      //std::cout << thread_name << " hello!" << std::endl;
      //std::cout.flush();
      try {
        custom_worker(terminating);
      } catch (const std::exception& e) {
        exception_thrown = true;
        last_exception = e;
        std::cout << thread_name << "::worker(), exception generated: \""<< e.what() << "\"" << std::endl;
        std::cout.flush();
      }
      terminating = true;
      //std::cout << thread_name << " goodbye :(" << std::endl;
      //std::cout.flush();
      alive = false;
      cond_worker_stop.notify_all();
      custom_terminating_callback();
    }
};

}

#endif /* ALGORITHM_H_ */
