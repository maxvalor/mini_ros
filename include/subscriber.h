#ifndef SUBSCRIBER_H_
#define SUBSCRIBER_H_

#include <functional>

namespace mini_ros {

class Subscriber {
  std::function<void(void)> f;
  
public:
  Subscriber(std::function<void(void)> f) : f(f) {}
  Subscriber() : f(nullptr) {}
  void shutdown() {
    if (f != nullptr ) {
      f();
    }
  }
};

} /* mini_ros */

#endif
