#ifndef SERVICE_CLIENT_H_
#define SERVICE_CLIENT_H_

#include "service.h"

namespace mini_ros {

class ServiceClient {
private:
  std::function<bool(Service*)> f;
  std::function<bool(std::shared_ptr<Service>)> f_sp;

public:
  ServiceClient(std::function<bool(Service*)> f,
    std::function<bool(std::shared_ptr<Service>)> f_sp) : f(f), f_sp(f_sp) {}
  ServiceClient() : f(nullptr), f_sp(nullptr) {}
  virtual ~ServiceClient () {}

  template <typename T>
  bool call(std::shared_ptr<T> srv) {
    if (f_sp != nullptr ) {
      return f_sp(srv);
    }

    return false;
  }

  template <typename T>
  bool call(T& srv)
  {
    if (f != nullptr ) {
      return f(dynamic_cast<Service*>(&srv));
    }

    return false;
  }
};

} /* mini_ros */

#endif
