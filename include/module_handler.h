#ifndef MODULE_HANDLER_H_
#define MODULE_HANDLER_H_

#include <thread>
#include "message_queue.h"
#include "core.h"
#include "subscriber.h"
#include "publisher.h"

namespace mini_ros {

class ModuleHandler {
private:
  struct function_pair {
    std::function<void(std::shared_ptr<Message>)> f;
    bool enable;
  };
  using func_vector = std::vector<function_pair>;
  std::map<std::string, func_vector> topic_callbacks;

  MessageQueue recv_msg_queue;
  MessageQueue send_msg_queue;
  bool running;
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

    std::thread::id tid = std::this_thread::get_id();
    auto push_back = [this](MessageQueue::MessagePair &msg)
    {
      recv_msg_queue.push(msg);
      recv_msg_queue.notify();
    };
    Core::instance().register_handler(tid, push_back);
  }

  virtual ~ModuleHandler () {}

  void spin()
  {
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
  Subscriber subscribe(std::string topic, std::function<void(std::shared_ptr<T> msg)> f) {
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
};

} /* mini_ros */

#endif
