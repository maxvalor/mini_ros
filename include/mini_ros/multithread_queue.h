#ifndef MULTI_THREAD_SAFE_QUEUE_H_
#define MULTI_THREAD_SAFE_QUEUE_H_

#include <queue>
#include <condition_variable>
#include <mutex>
#include <memory>

namespace mini_ros {

template <typename T>
class MultiThreadQueue {

public:
  MultiThreadQueue() : max_size(0) {}
  T front()
  {
    std::lock_guard<std::mutex> l(mtx);
    return msg_queue.front();
  }
  void emplace(T &msg)
  {
    std::lock_guard<std::mutex> l(mtx);
    // to make it faster
    if (max_size == 0 || msg_queue.size() + 1 <= max_size)
    {
      msg_queue.emplace(msg);
    }
    else
    {
      msg_queue.pop();
      msg_queue.emplace(msg);
    }
  }
  MultiThreadQueue(size_t max_size) : max_size(max_size)
  {
  }

  void pop() {
    std::lock_guard<std::mutex> l(mtx);
    msg_queue.pop();
  }

  bool empty() {
    std::lock_guard<std::mutex> l(mtx);
    return msg_queue.size() == 0;
  }

  void wait() {
    std::unique_lock <std::mutex> lck(mtx_cv);
    cv.wait(lck);
  }

  void notify() {
    cv.notify_one();
  }

private:
  std::queue<T> msg_queue;
  std::condition_variable cv;
  std::mutex mtx;
  std::mutex mtx_cv;
  size_t max_size;
};
}
#endif
