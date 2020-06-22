#ifndef MINI_ROS_H_
#define MINI_ROS_H_

#include <functional>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <cstring>
#include <future>
#include "message.h"

#include <iostream>

namespace mini_ros {

  class ModuleHandler {
    using func_vector = std::vector<std::function<void(std::shared_ptr<Message>)>>;
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
    void subscribe(std::string topic, std::function<void(std::shared_ptr<T> msg)> f) {
      auto packed_f = [f](std::shared_ptr<Message> msg){
        f(std::static_pointer_cast<T>(msg));
      };
      try {
        auto& funcs = topic_callbacks.at(topic);
        //std::cout << "func size:" << funcs.size() << std::endl;
        funcs.push_back(packed_f);
        // topic_callbacks.insert(
        //   std::pair<std::string, func_vector>(topic, funcs));
      }
      catch (std::out_of_range e) {
        func_vector funcs;
        funcs.push_back(packed_f);
        topic_callbacks.insert(
          std::pair<std::string, func_vector>(topic, funcs));
      }
    }

    template <typename T>
    void publish(std::string topic, T& msg) {
      try {
        // std::cout << "a1" << std::endl;
        auto funcs = topic_callbacks.at(topic);
        // std::cout << "a2" << std::endl;
        T* pMsg = new T();
        memcpy(pMsg, &msg, sizeof(T));
        std::shared_ptr<Message> sMsg(dynamic_cast<Message*>(pMsg));
        for (auto f : funcs) {
            // std::thread([f, sMsg](){
            //   f(sMsg);
            // });
            f(sMsg);
            // std::cout << "a4" << std::endl;
            // 为了限制线程数据，也可以使用线程池来进行一定的控制。
        }
      }
      catch (std::out_of_range e) {
        // do nothing.
        std::cout << "error" << std::endl;
      }
    }

};

  ModuleHandler* ModuleHandler::singleton = nullptr;
}

#endif
