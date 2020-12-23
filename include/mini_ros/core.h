#ifndef CORE_H_
#define CORE_H_

#include <map>
#include <list>
#include <thread>
#include "message_queue.h"
#include "service.h"

namespace mini_ros {

class Core {
private:
  std::map<std::thread::id,
    std::function<void(MessagePair)>> emplaces;

  std::map<std::string, std::list<std::thread::id>> subscribers;
  std::map<std::string, std::function<bool(std::shared_ptr<Service>)>> services;

  static Core *singleton;
  std::mutex emplaces_mtx;
  std::mutex subscribers_mtx;
  std::mutex services_mtx;

  Core () {}
public:

  static Core& instance();
  virtual ~Core () {}

  void register_handle(std::thread::id tid,
    std::function<void(MessagePair)> f);

  void subscribe(std::thread::id tid, std::string topic);

  void deliver(MessagePair msg);

  void register_service(std::string srv_name,
    std::function<bool(std::shared_ptr<Service>)> f);

  bool call_service(std::string srv_name, std::shared_ptr<Service> srv);
};

} /* mini_ros */
#endif
