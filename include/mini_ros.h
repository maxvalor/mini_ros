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
  //
  // class Subscriber {
  //   std::string topic;
  //   const ;
  //
  // public:
  //   Subscriber() {}
  //   ~Subscriber() {}
  //   Subscriber(std::string topic, const std::type_info &nInfo) : topic(topic), nInfo(nInfo) {}
  //   bool shutdown() {
  //     ModuleHandler& mh = ModuleHandler::instance();
  //     mh.remove(topic, nInfo);
  //   }
  // };
  template <typename T>
  class Subscriber;
  template <typename T>
  class Publisher;

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
    Subscriber<T> subscribe(std::string topic, std::function<void(std::shared_ptr<T> msg)> f) {
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
        remove(topic, index);
      };

      return Subscriber<T>(shutdown_f);
    }

    template <typename T>
    void publish(std::string topic, T& msg) {
      try {
        auto funcs = topic_callbacks.at(topic);
        T* pMsg = new T();
        memcpy(pMsg, &msg, sizeof(T));
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
    }

    template <typename T>
    Publisher<T> advertise(std::string topic) {
      return Publisher<T>(topic);
    }

    bool remove(std::string topic, std::uint32_t index) {
      try {
        auto &funcs = topic_callbacks.at(topic);
        //funcs.erase(funcs.begin() + index);
        funcs[index].enable = false;
      }
      catch (std::out_of_range e) {
        // do nothing.
        std::cout << "error" << std::endl;
      }
    }

  };

  template <typename T>
  class Publisher {
    //ModuleHandler &mh;
    std::string topic;

  public:
    Publisher(std::string topic) : topic(topic) {
      //mh = std::ref(mini_ros::ModuleHandler::instance());
    }
    void publish(T& msg) {
      ModuleHandler::instance().publish<T>(topic, msg);
    }
  };

  template <typename T>
  class Subscriber {
    std::function<void(void)> f;
  public:
    Subscriber() {}
    Subscriber(std::function<void(void)> f) : f(f) {}

    void shutdown() {
      f();
    }
  };

  ModuleHandler* ModuleHandler::singleton = nullptr;
}

#endif
