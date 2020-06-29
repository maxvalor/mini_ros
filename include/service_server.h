#ifndef SERVICE_SERVER_H_
#define SERVICE_SERVER_H_

namespace mini_ros {

class ServiceServer {
private:
  std::function<void(void)> f;

public:
  ServiceServer(std::function<void(void)> f) : f(f) {}
  ServiceServer() : f(nullptr) {}
  virtual ~ServiceServer () {}
  void shutdown() {
    if (f != nullptr ) {
      f();
    }
  }
};

} /* mini_ros */

#endif
