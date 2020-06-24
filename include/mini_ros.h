#ifndef MINI_ROS_H_
#define MINI_ROS_H_

#include <functional>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <cstring>
#include <future>
#include <typeinfo>
#include <unistd.h>
#include "message_queue.h"

#include <iostream>

namespace mini_ros {

  class ModuleHandler;

  class Publisher {
    using pub_func = std::function<void(Message*)>;
    using pub_func_sp = std::function<void(std::shared_ptr<Message>)>;
    pub_func f;
    pub_func_sp f_sp;
    friend class ModuleHandler;

    Publisher(pub_func f, pub_func_sp f_sp) : f(f), f_sp(f_sp) {}
  public:
    Publisher() : f(nullptr), f_sp(nullptr) {}
    template <typename T>
    void publish(T& msg) {
      if (f != nullptr) {
        f(&msg);
      }
    }

    template <typename T>
    void publish(std::shared_ptr<T> msg) {
      if (f_sp != nullptr) {
        f_sp(msg);
      }
    }
  };

  //template <typename T>
  class Subscriber {
    std::function<void(void)> f;
    friend class ModuleHandler;

    Subscriber(std::function<void(void)> f) : f(f) {}
  public:
    Subscriber() : f(nullptr) {}
    void shutdown() {
      if (f != nullptr ) {
        f();
      }
    }
  };

  // class ServiceServer {};
  // class ServerClient {};

  class ModuleHandler {
    struct function_pair {
      std::function<void(std::shared_ptr<Message>)> f;
      bool enable;
    };
    using func_vector = std::vector<function_pair>;
    std::map<std::string, func_vector> topic_callbacks;
    MessageQueue msg_queue;
    static ModuleHandler* singleton;

    ModuleHandler() {

      std::thread deliver_t = std::thread([this](){
        while (1) {
          if (!msg_queue.empty()) {
            MessageQueue::MessagePair pair = msg_queue.front();
            auto& funcs = topic_callbacks.at(pair.first);
            std::shared_ptr<Message> sMsg = pair.second;
            for (auto fp : funcs) {
              std::thread t = std::thread([fp, sMsg](){
                if (fp.enable) {
                    fp.f(sMsg);
                }
              });
              t.detach();
              // 为了限制线程数据，也可以使用线程池来进行一定的控制。
            }
            msg_queue.pop();
          }
          else {
            msg_queue.wait();
          }
        }
      });

      deliver_t.detach();
    }

  public:
    static ModuleHandler& instance() {
      static std::mutex mtx;

      if (singleton == nullptr) {
        mtx.lock();
        if (singleton == nullptr) {
          singleton = new ModuleHandler();
        }
        mtx.unlock();
      }

      return *singleton;
    }

    template <typename T>
    Subscriber subscribe(std::string topic, std::function<void(std::shared_ptr<T> msg)> f) {
      auto packed_f = [f](std::shared_ptr<Message> msg){
        f(std::static_pointer_cast<T>(msg));
      };
      std::cout << typeid(packed_f).name() << std::endl;
      size_t index;
      try {
        auto& funcs = topic_callbacks.at(topic);
        function_pair fp = {packed_f, true};
        funcs.push_back(fp);
        index = funcs.size() - 1;
      }
      catch (std::out_of_range e) {
        func_vector funcs;
        function_pair fp = {packed_f, true};
        funcs.push_back(fp);
        index = funcs.size() - 1;
        topic_callbacks.insert(
          std::pair<std::string, func_vector>(topic, funcs));
      }

      auto shutdown_f = [this, index, topic] {
        try {
          auto &funcs = topic_callbacks.at(topic);
          //funcs.erase(funcs.begin() + index);
          funcs[index].enable = false;
        }
        catch (std::out_of_range e) {
          // do nothing.
          std::cout << "no subscriber" << std::endl;
        }
      };

      return Subscriber(shutdown_f);
    }

    template <typename T>
    Publisher advertise(std::string topic) {
      auto publish_f = [this, topic](Message* msg){
        try {
          auto funcs = topic_callbacks.at(topic);
          msg = new T(std::move(*dynamic_cast<T*>(msg)));
          std::shared_ptr<Message> sMsg(msg);
          MessageQueue::MessagePair p(topic, sMsg);
          msg_queue.push(p);
          msg_queue.notify();
        }
        catch (std::out_of_range e) {
          // do nothing.
          std::cout << "no subscriber" << std::endl;
        }
      };

      auto publish_sp_f = [this, topic](std::shared_ptr<Message> msg){
        try {
          auto funcs = topic_callbacks.at(topic);
          std::shared_ptr<T> sMsg = std::static_pointer_cast<T>(msg);
          MessageQueue::MessagePair p(topic, sMsg);
          msg_queue.push(p);
          msg_queue.notify();
        }
        catch (std::out_of_range e) {
          // do nothing.
          std::cout << "no subscriber" << std::endl;
        }
      };

      return Publisher(publish_f, publish_sp_f);
    }
    //
    // template <typename T>
    // ServiceServer advertiseService(std::string name,
    //   std::function<void(std::shared_ptr<typename T::Request>,
    //     std::shared_ptr<typename T::Response>)> f) {
    //
    //   return ServiceServer();
    // }

  };

  ModuleHandler* ModuleHandler::singleton = nullptr;
}

#endif
