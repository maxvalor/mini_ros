#ifndef MINI_ROS_H_
#define MINI_ROS_H_

#include "module_handler.h"
#include "module.h"

// for test
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdarg.h>

namespace mini_ros {

struct init
{
  void operator()()
  {
    Core::instance();
  }
};

void __mini_ros_stop(int signo)
{
    _exit(0);
}

bool hold(size_t count, ...)
{
  va_list ap;
  va_start(ap, count);

  signal(SIGINT, __mini_ros_stop);

  for (int i = 0; i < count; ++i) {
    Module *pModule = va_arg(ap, Module*);
    pModule->wait();
  }

  va_end(ap);

  return true;
}

// struct hold
// {
//   bool operator() (size_t count, ...)
//   {
//     va_list ap;
//     va_start(ap, count);
//
//     signal(SIGINT, __mini_ros_stop);
//
//     for (int i = 0; i < count; ++i) {
//       Module *pModule = va_arg(ap, Module*);
//       modules.push_back(pModule);
//     }
//
//     for (auto module : modules)
//     {
//       module->wait();
//     }
//
//     va_end(ap);
//
//     return true;
//   }
// };




} /* mini_ros */

#endif
