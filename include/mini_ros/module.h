#ifndef MODULE_H_
#define MODULE_H_

#include <thread>
#include "module_handler.h"

namespace mini_ros {

class Module {

public:
  void wait();
  void stop();
  void sleep(uint32_t ms);
  virtual void onInit() = 0;
  virtual void onExit();
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
