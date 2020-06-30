#ifndef MODULE_HANDLER_H_
#define MODULE_HANDLER_H_

#include <thread>
#include <assert.h>
#include <future>
#include <chrono>
#include "message_queue.h"
#include "core.h"
#include "subscriber.h"
#include "publisher.h"
#include "service_server.h"
#include "service_client.h"
#include "service.h"

namespace mini_ros {

class ModuleHandler;

using ThreadHandler = ModuleHandler;

class ModuleHandler {
private:
  struct function_pair {
    std::function<void(std::shared_ptr<Message>)> f;
    bool enable;
  };
  using func_vector = std::vector<function_pair>;
  std::map<std::string, func_vector> topic_callbacks;
  struct service_pair {
    std::function<bool(std::shared_ptr<Service>)> f;
    bool enable;
  };
  std::map<std::string, service_pair> service_funcs;

  MessageQueue recv_msg_queue;
  MessageQueue send_msg_queue;
  bool running;
  std::thread::id tid;

public:
  ModuleHandler()
  {
    std::thread deliver_t = std::thread([this](){
      while (1) {
        if (!send_msg_queue.empty()) {
          MessageQueue::MessagePair pair = send_msg_queue.front();
          Core::instance().deliver(pair);
          send_msg_queue.pop();
        }
        else {
          send_msg_queue.wait();
        }
      }
    });

    deliver_t.detach();

    tid = std::this_thread::get_id();
    auto push_back = [this](MessageQueue::MessagePair msg)
    {
      recv_msg_queue.push(msg);
      recv_msg_queue.notify();
    };
    Core::instance().register_handler(tid, push_back);
  }

  virtual ~ModuleHandler () {}

  void spin()
  {
    assert(tid == std::this_thread::get_id());
    running = true;
    while (running)
    {
      if (!recv_msg_queue.empty())
      {
        MessageQueue::MessagePair pair = recv_msg_queue.front();
        auto& funcs = topic_callbacks.at(pair.first);
        std::shared_ptr<Message> sMsg = pair.second;
        for (auto fp : funcs)
        {
          if (fp.enable)
          {
              fp.f(sMsg);
          }
        }
        recv_msg_queue.pop();
      }
      else
      {
        recv_msg_queue.wait();
      }
    }
  }

  void stop()
  {
    running = false;
  }

  template <typename T>
  Subscriber subscribe(std::string topic, std::function<void(std::shared_ptr<T>)> f) {

    auto packed_f = [f](std::shared_ptr<Message> msg)
    {
      f(std::static_pointer_cast<T>(msg));
    };

    //std::cout << typeid(packed_f).name() << std::endl;
    size_t index;
    try
    {
      auto& funcs = topic_callbacks.at(topic);
      function_pair fp = {packed_f, true};
      funcs.push_back(fp);
      index = funcs.size() - 1;
    }
    catch (std::out_of_range e)
    {
      func_vector funcs;
      function_pair fp = {packed_f, true};
      funcs.push_back(fp);
      index = funcs.size() - 1;
      topic_callbacks.insert(
        std::pair<std::string, func_vector>(topic, funcs));
    }

    Core::instance().subscribe(std::this_thread::get_id(), topic);

    auto shutdown_f = [this, index, topic]
    {
      try
      {
        auto &funcs = topic_callbacks.at(topic);
        //funcs.erase(funcs.begin() + index);
        funcs[index].enable = false;
      }
      catch (std::out_of_range e)
      {
        // do nothing.
        std::cout << "no subscriber" << std::endl;
      }
    };

    return Subscriber(shutdown_f);
  }

  template <typename T>
  Publisher advertise(std::string topic) {
    auto publish_f = [this, topic](Message* msg){
      msg = new T(std::move(*dynamic_cast<T*>(msg)));
      std::shared_ptr<Message> sMsg(msg);
      MessageQueue::MessagePair p(topic, sMsg);
      send_msg_queue.push(p);
      send_msg_queue.notify();
    };

    auto publish_sp_f = [this, topic](std::shared_ptr<Message> msg){
      std::shared_ptr<T> sMsg = std::static_pointer_cast<T>(msg);
      MessageQueue::MessagePair p(topic, sMsg);
      send_msg_queue.push(p);
      send_msg_queue.notify();
    };

    return Publisher(publish_f, publish_sp_f);
  }

  template <typename T>
  ServiceServer advertiseService(std::string srv_name,
    std::function<bool(std::shared_ptr<T>)> f, bool sync = false)
  {
    if (sync)
    {
      std::shared_ptr<std::mutex> pmtx = std::make_shared<std::mutex>();
      auto sync_f = [f, pmtx](std::shared_ptr<T> srv)
      {
        bool rlt;
        pmtx->lock();
        rlt = f(srv);
        pmtx->unlock();
        return rlt;
      };

      f = sync_f;
    }

    auto packed_f = [f](std::shared_ptr<Service> srv){
      std::shared_ptr<T> t_srv = std::static_pointer_cast<T>(srv);
      return f(t_srv);
    };

    size_t index;
    // try
    // {
    //   auto& funcs = service_funcs.at(srv_name);
    //   service_pair fp = {packed_f, true};
    //   funcs.push_back(fp);
    //   index = funcs.size() - 1;
    // }
    // catch (std::out_of_range e)
    // {
    //   srv_vector funcs;
    //   service_pair fp = {packed_f, true};
    //   funcs.push_back(fp);
    //   index = funcs.size() - 1;
    //   service_funcs.insert(
    //     std::pair<std::string, srv_vector>(srv_name, funcs));
    // }

    //service_pair sp = {packed_f, true};
    Core::instance().register_service(srv_name, packed_f);

    auto shutdown_f = [srv_name]
    {
      std::cout << "succeed in down." << std::endl;
      auto return_false = [](std::shared_ptr<Service> srv)
      {
        return false;
      };
      Core::instance().register_service(srv_name, return_false);
    };

    return ServiceServer(shutdown_f);
  }

  template <typename T>
  ServiceClient serviceClient(std::string srv_name, std::uint32_t timeout_ms = 0)
  {
    auto call = [srv_name, timeout_ms](Service* origin_srv)
    {
      Service* osrv = dynamic_cast<Service*>(new T(std::move(*static_cast<T*>(origin_srv))));
      std::shared_ptr<Service> srv(osrv);
      if (timeout_ms > 0)
      {
        std::future<bool> result = std::async([srv_name, origin_srv, srv](){
          bool rlt = Core::instance().call_service(srv_name, srv);
          std::shared_ptr<T> t_srv = std::static_pointer_cast<T>(srv);
          static_cast<T*>(origin_srv)->resp = t_srv->resp;
          return rlt;
        });
        try
        {
          std::chrono::system_clock::time_point two_seconds_passed
              = std::chrono::system_clock::now() + std::chrono::milliseconds(timeout_ms);
          result.wait_until(two_seconds_passed);
          return result.get();
        }
        catch (...)
        {
            std::cout << "get error....\n ";
        }
      }
      else
      {
        bool rlt = Core::instance().call_service(srv_name, srv);
        std::shared_ptr<T> t_srv = std::static_pointer_cast<T>(srv);
        static_cast<T*>(origin_srv)->resp = t_srv->resp;
        return rlt;
      }

      return false;
    };

    auto call_sp = [srv_name, timeout_ms](std::shared_ptr<Service> srv)
    {
      if (timeout_ms > 0)
      {
        std::future<bool> result = std::async([srv_name, srv](){
          return Core::instance().call_service(srv_name, srv);
        });
        try
        {
          std::chrono::system_clock::time_point two_seconds_passed
              = std::chrono::system_clock::now() + std::chrono::milliseconds(timeout_ms);
          result.wait_until(two_seconds_passed);
          return result.get();
        }
        catch (...)
        {
            std::cout << "get error....\n ";
        }
      }
      else
      {
        return Core::instance().call_service(srv_name, srv);
      }

      return false;
    };

    return ServiceClient(call, call_sp);
  }
};

} /* mini_ros */

#endif
