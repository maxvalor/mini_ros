#ifndef MODULE_H_
#define MODULE_H_

#include <thread>
#include "module_handle.h"

namespace mini_ros {

class Module {

public:
  void wait();
  void stop();
  void sleep(uint32_t ms);
protected:
  ModuleHandle& getModuleHandle();

  virtual void onInit() = 0;
  virtual void onExit();
  virtual void onStopped();

private:
  std::thread _t;
  ModuleHandle* handle;
  std::condition_variable cv;
  std::mutex mtx;
public:
  Module();
  virtual ~Module();
};

} /* mini_ros */

#endif
