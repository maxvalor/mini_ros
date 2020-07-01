#ifndef SERVICE_CLIENT_H_
#define SERVICE_CLIENT_H_

#include "service.h"

namespace mini_ros {

class ServiceClient {
private:
  std::function<bool(std::shared_ptr<Service>)> f;

  friend class ThreadHandler;
  ServiceClient(std::function<bool(std::shared_ptr<Service>)> f) : f(f) {}
public:

  ServiceClient() : f(nullptr) {}
  virtual ~ServiceClient () {}

  template <typename T>
  bool call(std::shared_ptr<T> srv) {
    if (f != nullptr ) {
      return f(srv);
    }

    return false;
  }

  template <typename T>
  bool call(T& srv)
  {
    if (f != nullptr ) {
      bool rlt = false;
      std::shared_ptr<T> sSrv(new T(std::move(srv)));
      rlt = f(sSrv);
      srv.resp = sSrv->resp;
      return rlt;
    }

    return false;
  }
};

} /* mini_ros */

#endif
