#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#include <functional>
#include "message.h"

namespace mini_ros {

class Publisher {
  using pub_func = std::function<void(Message*)>;
  using pub_func_sp = std::function<void(std::shared_ptr<Message>)>;
  pub_func f;
  pub_func_sp f_sp;

public:
  Publisher(pub_func f, pub_func_sp f_sp) : f(f), f_sp(f_sp) {}
  Publisher() : f(nullptr), f_sp(nullptr) {}
  template <typename T>
  void publish(T& msg) {
    if (f != nullptr) {
      f(&msg);
    }
  }

  template <typename T>
  void publish(std::shared_ptr<T> msg) {
    if (f_sp != nullptr) {
      f_sp(msg);
    }
  }
};

} /* mini_ros */

#endif
