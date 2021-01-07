#ifndef MAIN_THREAD_H_
#define MAIN_THREAD_H_

#include <functional>
#include "multithread_queue.h"

namespace mini_ros {
  class MainThreadEvent
  {
  public:
    MainThreadEvent(std::function<void(void)> f) : f(f) {}
    std::function<void(void)> get_f()
    {
      return f;
    }
  private:
    std::condition_variable cv;
    std::mutex mtx;
    std::function<void(void)> f;

    friend class MainThreadProcessor;

    bool wait()
    {
      std::unique_lock <std::mutex> lck(mtx);
      cv.wait(lck);
    }

    void notify()
    {
      cv.notify_one();
    }
  };

  using MainThreadEventSP = std::shared_ptr<MainThreadEvent>;

  class MainThreadProcessor
  {
    static MainThreadProcessor* singleton;
    MainThreadProcessor() {}
  public:
    static MainThreadProcessor& instance() {
      static std::mutex mtx;
      if (singleton == nullptr) {
        mtx.lock();
        if (singleton == nullptr) {
          singleton = new MainThreadProcessor();
        }
        mtx.unlock();
      }

      return *singleton;
    }

    void addEvent(MainThreadEventSP& event)
    {
      event_queue.emplace(event);
      event_queue.notify();
    }

    void waitEvent(MainThreadEventSP& event)
    {
      addEvent(event);
      event->wait();
    }

    void processEvent()
    {
      running = true;
      while (running)
      {
        if (!event_queue.empty()) {
          MainThreadEventSP event = event_queue.front();
          event->f();
          event->notify();
          event_queue.pop();
        }
        else {
          event_queue.wait();
        }
      }
    }

    void stop()
    {
      running = false;
      event_queue.notify();
    }

  private:
    MultiThreadQueue<MainThreadEventSP> event_queue;
    bool running = false;
  };


  template <typename T>
  struct MainThreadObject
  {
  public:
    std::shared_ptr<T> operator()()
    {
      T* object;
      T** pObj = &object;
      std::shared_ptr<T> event(new MainThreadEvent([pObj]()
      {
        *pObj = new T();
      }));
      MainThreadProcessor::instance().waitEvent(event);
      return std::shared_ptr<T>(object);
    }
  };




} /* mini_ros */

#endif
