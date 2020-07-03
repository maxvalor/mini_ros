#include "message_queue.h"
#include <memory>

namespace mini_ros {

  MessageQueue::MessageQueue(size_t max_size) : max_size(max_size)
  {
  }

  MessageQueue::MessagePair MessageQueue::front() {
    std::lock_guard<std::mutex> l(mtx);
    return msg_queue.front();
  }
  void MessageQueue::pop() {
    std::lock_guard<std::mutex> l(mtx);
    msg_queue.pop();
  }
  void MessageQueue::push(MessagePair &msg) {
    std::lock_guard<std::mutex> l(mtx);
    // to make it faster
    if (max_size == 0 || msg_queue.size() + 1 <= max_size)
    {
      msg_queue.push(msg);
    }
    else
    {
      msg_queue.pop();
      msg_queue.push(msg);
    }
  }

  bool MessageQueue::empty() {
    std::lock_guard<std::mutex> l(mtx);
    return msg_queue.size() == 0;
  }

  void MessageQueue::wait() {
    std::unique_lock <std::mutex> lck(mtx_cv);
    cv.wait(lck);
  }

  void MessageQueue::notify() {
    cv.notify_one();
  }
}
