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
  std::mutex push_backs_mtx;
  std::mutex subscribers_mtx;
  std::mutex services_mtx;

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
    std::lock_guard<std::mutex> lck(push_backs_mtx);
    push_backs.insert(std::pair<std::thread::id,
      std::function<void(MessageQueue::MessagePair)>>(tid, f));

    std::cout << "register_handler:" << tid << std::endl;
  }

  void subscribe(std::thread::id tid, std::string topic)
  {
    size_t index;
    std::lock_guard<std::mutex> lck(subscribers_mtx);
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
      std::lock_guard<std::mutex> lck(subscribers_mtx);
      auto& tids = subscribers.at(msg.first);
      for (auto tid : tids)
      {
        try {
          std::lock_guard<std::mutex> lck(push_backs_mtx);
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
    std::lock_guard<std::mutex> lck(services_mtx);
    try
    {
      auto& f_n = services.at(srv_name);
      f_n = f;
    }
    catch (std::out_of_range e)
    {
      services.insert(std::pair<std::string,
          std::function<bool(std::shared_ptr<Service>)>>(srv_name, f));
    }
  }

  bool call_service(std::string srv_name,
    std::shared_ptr<Service> srv)
  {
    try
    {
      services_mtx.lock();
      auto f = services.at(srv_name);
      services_mtx.unlock();
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
