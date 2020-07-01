#include "core.h"

namespace mini_ros {

  Core* Core::singleton = nullptr;

  Core& Core::instance() {
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

  void Core::register_handler(std::thread::id tid,
    std::function<void(MessageQueue::MessagePair)> f)
  {
    std::lock_guard<std::mutex> lck(push_backs_mtx);
    push_backs.insert(std::pair<std::thread::id,
      std::function<void(MessageQueue::MessagePair)>>(tid, f));
  }

  void Core::subscribe(std::thread::id tid, std::string topic)
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

  void Core::deliver(MessageQueue::MessagePair msg)
  {
    std::lock_guard<std::mutex> lck(subscribers_mtx);
    std::lock_guard<std::mutex> lck2(push_backs_mtx);
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
    }
  }

  void Core::register_service(std::string srv_name,
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

  bool Core::call_service(std::string srv_name,
    std::shared_ptr<Service> srv)
  {
    bool found = false;
    try
    {
      services_mtx.lock();
      auto f = services.at(srv_name);
      services_mtx.unlock();
      return f(srv);
    }
    catch (std::out_of_range e)
    {
      services_mtx.unlock();
    }
    return false;
  }

} /* mini_ros */
