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
#include "message.h"

#include <iostream>

namespace mini_ros {

  class ModuleHandler;

  class Publisher {
    std::function<void(Message* msg)> f;
  public:
    Publisher(std::function<void(Message* msg)> f) : f(f) {
    }
    template <typename T>
    void publish(T& msg) {
      T* pMsg = &msg;
      f(static_cast<Message*>(pMsg));
    }
  };

  //template <typename T>
  class Subscriber {
    std::function<void(void)> f;
  public:
    Subscriber() {}
    Subscriber(std::function<void(void)> f) : f(f) {}

    void shutdown() {
      f();
    }
  };

  class ModuleHandler {
    struct function_pair {
      std::function<void(std::shared_ptr<Message>)> f;
      bool enable;
    };
    using func_vector = std::vector<function_pair>;
    std::map<std::string, func_vector> topic_callbacks;
    static ModuleHandler* singleton;

    ModuleHandler() {}

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
        //std::cout << "func size:" << funcs.size() << std::endl;
        function_pair fp = {packed_f, true};
        funcs.push_back(fp);
        index = funcs.size() - 1;
        // topic_callbacks.insert(
        //   std::pair<std::string, func_vector>(topic, funcs));
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
          std::cout << "error" << std::endl;
        }
      };

      return Subscriber(shutdown_f);
    }

    template <typename T>
    Publisher advertise(std::string topic) {
      auto publish_f = [this, topic](Message* msg){
        try {
          auto funcs = topic_callbacks.at(topic);
          T* n_msg = static_cast<T*>(msg);
          T* pMsg = new T();
          memcpy(pMsg, n_msg, sizeof(T));
          std::shared_ptr<Message> sMsg(dynamic_cast<Message*>(pMsg));
          for (auto fp : funcs) {
              std::thread t = std::thread([fp, sMsg](){
                if (fp.enable) {
                    fp.f(sMsg);
                }
              });
              t.detach();
              // 为了限制线程数据，也可以使用线程池来进行一定的控制。
          }
        }
        catch (std::out_of_range e) {
          // do nothing.
          std::cout << "error" << std::endl;
        }
      };

      return Publisher(publish_f);
    }

  };

  ModuleHandler* ModuleHandler::singleton = nullptr;
}

#endif
