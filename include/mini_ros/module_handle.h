#ifndef MODULE_HANDLE_H_
#define MODULE_HANDLE_H_

#include "thread_handle.h"

namespace mini_ros {

  class ModuleHandle : public ThreadHandle
  {
  private:
    friend class Module;
    ModuleHandle() {}
  };

} /* mini_ros */

#endif
