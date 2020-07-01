#ifndef MODULE_H_
#define MODULE_H_

#include <thread>
#include "module_handler.h"

namespace mini_ros {

class Module {

public:
  void wait();
  void sleep(uint32_t ms);
  virtual void run() = 0;
protected:
  ModuleHandler& getModuleHandler();

private:
  std::thread _t;
  ModuleHandler* handler;
public:
  Module();
  virtual ~Module();
};

} /* mini_ros */

#endif
