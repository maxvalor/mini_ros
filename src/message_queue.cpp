#include "message_queue.h"

namespace mini_ros {

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
    msg_queue.push(msg);
  }

  bool MessageQueue::empty() {
    std::lock_guard<std::mutex> l(mtx);
    return msg_queue.size() == 0;
  }

  void MessageQueue::wait() {
    std::unique_lock <std::mutex> lck(mtx2);
    cv.wait(lck);
  }

  void MessageQueue::notify() {
    cv.notify_one();
  }
}
