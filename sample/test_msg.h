#ifndef TEST_MSG_H_
#define TEST_MSG_H_
#include "message.h"
#include <iostream>
struct TestMsg : public mini_ros::Message {
  std::uint32_t data;
  ~TestMsg() { std::cout << "~TestMsg" << std::endl; }
};
#endif
