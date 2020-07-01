#ifndef MODULE_HANDLER_H_
#define MODULE_HANDLER_H_

#include "thread_handler.h"

namespace mini_ros {

  class ModuleHandler : public ThreadHandler
  {
  private:
    friend class Module;
    ModuleHandler() {}
  };

} /* mini_ros */

#endif
