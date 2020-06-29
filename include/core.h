#ifndef CORE_H_
#define CORE_H_

#include <thread>
#include <map>
#include <list>
#include "message_queue.h"
#include "service.h"

#include <iostream>

namespace mini_ros {



class Core {
private:
  std::map<std::thread::id,
    std::function<void(MessageQueue::MessagePair)>> push_backs;

  std::map<std::string, std::list<std::thread::id>> subscribers;
  std::map<std::string, std::function<bool(std::shared_ptr<Service>)>> services;

  static Core *singleton;
  std::mutex rmtx;

  Core () {}
public:

  static Core& instance() {
    static std::mutex mtx;
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
    std::function<void(MessageQueue::MessagePair)> f)
  {
    rmtx.lock();
    push_backs.insert(std::pair<std::thread::id,
      std::function<void(MessageQueue::MessagePair)>>(tid, f));

    std::cout << "register_handler:" << tid << std::endl;
    rmtx.unlock();
  }

  void subscribe(std::thread::id tid, std::string topic)
  {
    size_t index;
    try {
      auto& thread_ids = subscribers.at(topic);
      for (auto id : thread_ids)
      {
        if (id == tid)
        {
          return;
        }
      }
      thread_ids.push_back(tid);
    }
    catch (std::out_of_range e) {
      std::list<std::thread::id> thread_ids;
      thread_ids.push_back(tid);
      subscribers.insert(
        std::pair<std::string, std::list<std::thread::id>>(topic, thread_ids));
    }
  }

  void deliver(MessageQueue::MessagePair msg)
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

  void register_service(std::string srv_name,
    std::function<bool(std::shared_ptr<Service>)> f)
  {
    services.insert(std::pair<std::string,
        std::function<bool(std::shared_ptr<Service>)>>(srv_name, f));
  }

  bool call_service(std::string srv_name,
    std::shared_ptr<Service> srv)
  {
    try
    {
      auto& f = services.at(srv_name);
      std::cout << "service found" << std::endl;
      return f(srv);
    }
    catch (std::out_of_range e)
    {
      std::cout << "no service" << std::endl;
    }
    return false;
  }
};

Core* Core::singleton = nullptr;

} /* mini_ros */
#endif
