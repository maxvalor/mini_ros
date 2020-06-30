#ifndef MINI_ROS_H_
#define MINI_ROS_H_

#include "module_handler.h"
#include "module.h"

namespace mini_ros {

struct init
{
  void operator()(...)
  {
    Core::instance();
  }
};

} /* mini_ros */

#endif
