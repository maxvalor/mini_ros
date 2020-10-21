#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include <functional>
#include "message.h"

namespace mini_ros {

class Publisher {
  using pub_func = std::function<void(std::shared_ptr<Message>, const char*)>;
  pub_func f;

  friend class ThreadHandle;
  Publisher(pub_func f) : f(f) {}

public:
  Publisher() : f(nullptr) {}
  template <typename T>
  void publish(T& msg) {
    if (f != nullptr) {
      std::shared_ptr<Message> sMsg(new T(std::move(msg)));
      f(sMsg, typeid(T).name());
    }
  }

  template <typename T>
  void publish(std::shared_ptr<T> msg) {
    if (f != nullptr) {
      f(msg, typeid(T).name());
    }
  }
};

} /* mini_ros */

#endif
