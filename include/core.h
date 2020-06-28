#ifndef CORE_H_
#define CORE_H_

#include <thread>
#include <map>
#include <list>
#include "message_queue.h"

#include <iostream>

namespace mini_ros {

std::mutex mtx;

class Core {
private:
  std::map<std::thread::id,
    std::function<void(MessageQueue::MessagePair&)>> push_backs;

  std::map<std::string, std::list<std::thread::id>> subscribers;

  static Core *singleton;
  std::mutex rmtx;

  Core () {}
public:

  static Core& instance() {

    if (singleton == nullptr) {
      mtx.lock();
      if (singleton == nullptr) {
        singleton = new Core();
      }
      mtx.unlock();
    }

    return *singleton;
  }
  virtual ~Core () {}

  void register_handler(std::thread::id tid,
    std::function<void(MessageQueue::MessagePair&)> f)
  {
    rmtx.lock();
    push_backs.insert(std::pair<std::thread::id,
      std::function<void(MessageQueue::MessagePair&)>>(tid, f));

    std::cout << "register_handler:" << tid << std::endl;
    rmtx.unlock();
  }

  void subscribe(std::thread::id tid, std::string& topic)
  {
    size_t index;
    try {
      auto& thread_ids = subscribers.at(topic);
      try {
        thread_ids.push_back(tid);
      }
      catch (std::out_of_range e) {
        // do nothing
      }
    }
    catch (std::out_of_range e) {
      std::list<std::thread::id> thread_ids;
      thread_ids.push_back(tid);
      subscribers.insert(
        std::pair<std::string, std::list<std::thread::id>>(topic, thread_ids));
    }
  }

  void deliver(MessageQueue::MessagePair& msg)
  {
    try {
      auto& tids = subscribers.at(msg.first);
      for (auto tid : tids)
      {
        try {
          auto push_back = push_backs.at(tid);
          push_back(msg);
        }
        catch (std::out_of_range e) {
          // do nothing
        }
      }
    }
    catch (std::out_of_range e) {
      std::cout << "no subscriber" << std::endl;
    }
  }
};

Core* Core::singleton = nullptr;

} /* mini_ros */
#endif
