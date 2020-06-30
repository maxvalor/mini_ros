#ifndef TEST_MODULE_H_
#define TEST_MODULE_H_

#include "mini_ros.h"

class TestModule : public mini_ros::Module
{
  void run() override
  {
    std::cout << "1" << std::endl;
  }
};

#endif
