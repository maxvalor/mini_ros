#include "thread_handle.h"
#include <iostream>

namespace mini_ros {

  ThreadHandle::ThreadHandle()
  {
    std::thread deliver_t = std::thread([this](){
      while (1) {
        if (!send_msg_queue.empty()) {
          MessagePair pair = send_msg_queue.front();
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
    auto push_back = [this](MessagePair msg)
    {
      recv_msg_queue.push(msg);
      recv_msg_queue.notify();
    };
    Core::instance().register_handle(tid, push_back);
  }

  ThreadHandle::~ThreadHandle () {}

  void ThreadHandle::spin()
  {
    assert(tid == std::this_thread::get_id());
    running = true;
    while (running)
    {
      if (!recv_msg_queue.empty())
      {
        MessagePair pair = recv_msg_queue.front();
        recv_msg_queue.pop();
        std::lock_guard<std::mutex>  lck(tcb_mtx);
        auto& funcs = topic_callbacks.at(pair.first);
        std::shared_ptr<Message> sMsg = pair.second;
        for (auto fp : funcs)
        {
          if (fp.enable)
          {
              fp.f(sMsg);
          }
        }
      }
      else
      {
        recv_msg_queue.wait();
      }
    }
  }

  void ThreadHandle::stop()
  {
    running = false;
    recv_msg_queue.notify();
  }

  Subscriber ThreadHandle::_subscribe(std::string& topic, std::function<void(std::shared_ptr<Message>)> f)
  {
    size_t index;
    tcb_mtx.lock();
    try
    {
      auto& funcs = topic_callbacks.at(topic);

      // erase false
      for (auto fp = funcs.begin(); fp != funcs.end();)
      {
        if (fp->enable == false)
        {
          funcs.erase(fp++);
        }
        else
        {
          ++fp;
        }
      }

      function_pair fp = {f, true};
      funcs.push_back(fp);
      index = funcs.size() - 1;
    }
    catch (std::out_of_range e)
    {
      func_vector funcs;
      function_pair fp = {f, true};
      funcs.push_back(fp);
      index = funcs.size() - 1;
      topic_callbacks.insert(
        std::pair<std::string, func_vector>(topic, funcs));
    }
    tcb_mtx.unlock();
    Core::instance().subscribe(std::this_thread::get_id(), topic);

    auto shutdown_f = [this, index, topic]
    {
      try
      {
        auto &funcs = topic_callbacks.at(topic);
        auto to_shutdown = funcs.begin();
        for (size_t i = 0; i < index; ++i)
        {
          ++to_shutdown;
        }
        to_shutdown->enable = false;
      }
      catch (std::out_of_range e)
      {
        // do nothing.
      }
    };

    return Subscriber(shutdown_f);
  }

  Publisher ThreadHandle::_advertise(std::string& topic, const char* tname)
  {
      auto publish_f = [this, topic, tname](std::shared_ptr<Message> msg, const char* msg_tname)
      {
        assert(tname == msg_tname);
        MessagePair p(topic, msg);
        send_msg_queue.push(p);
        send_msg_queue.notify();
      };

      return Publisher(publish_f);
  }

  ServiceServer ThreadHandle::_advertiseService(std::string& srv_name,
    std::function<bool(std::shared_ptr<Service>)> f, bool sync)
  {

    if (sync)
    {
      std::shared_ptr<std::mutex> pmtx = std::make_shared<std::mutex>();
      auto sync_f = [f, pmtx](std::shared_ptr<Service> srv)
      {
        bool rlt;
        pmtx->lock();
        rlt = f(srv);
        pmtx->unlock();
        return rlt;
      };

      f = sync_f;
    }

    Core::instance().register_service(srv_name, f);

    auto shutdown_f = [srv_name]
    {
      // std::cout << "succeed in down." << std::endl;
      auto return_false = [](std::shared_ptr<Service> srv)
      {
        return false;
      };
      Core::instance().register_service(srv_name, return_false);
    };

    return ServiceServer(shutdown_f);
  }

  ServiceClient ThreadHandle::_serviceClient(std::string& srv_name,
    std::uint32_t timeout_ms, const char* tname)
  {
    auto call = [srv_name, timeout_ms, tname](std::shared_ptr<Service> srv, const char* msg_tname)
    {
      assert(tname == msg_tname);
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
            // std::cout << "get error....\n ";
        }
      }
      else
      {
        return Core::instance().call_service(srv_name, srv);
      }

      return false;
    };

    return ServiceClient(call);
  }

} /* mini_ros */
