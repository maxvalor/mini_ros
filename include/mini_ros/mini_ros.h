#ifndef MINI_ROS_H_
#define MINI_ROS_H_

#include "module_handler.h"
#include "module.h"

// for test
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

namespace mini_ros {

void __mini_ros_stop(int signo)
{
    _exit(0);
}

struct init
{
  void operator()()
  {
    Core::instance();

    signal(SIGINT, __mini_ros_stop);
  }
};




} /* mini_ros */

#endif
