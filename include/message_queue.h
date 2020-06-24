#ifndef MESSAGE_QUEUE_H_
#define MESSAGE_QUEUE_H_

#include <queue>
#include <memory>
#include <condition_variable>
#include <mutex>
#include "message.h"

namespace mini_ros {

class MessageQueue {

public:
  using MessagePair = std::pair<std::string, std::shared_ptr<Message>>;
  MessagePair front() {
    std::lock_guard<std::mutex> l(mtx);
    return msg_queue.front();
  }
  void pop() {
    std::lock_guard<std::mutex> l(mtx);
    msg_queue.pop();
  }
  void push(MessagePair &msg) {
    std::lock_guard<std::mutex> l(mtx);
    msg_queue.push(msg);
  }

  bool empty() {
    std::lock_guard<std::mutex> l(mtx);
    return msg_queue.size() == 0;
  }

  void wait() {
    std::unique_lock <std::mutex> lck(mtx2);
    cv.wait(lck);
  }

  void notify() {
    cv.notify_one();
  }

private:
  std::queue<MessagePair> msg_queue;
  std::condition_variable cv;
  std::mutex mtx;
  std::mutex mtx2;
};
}
#endif
