#ifndef TEST_MSG_H_
#define TEST_MSG_H_
#include "message.h"
#include "tool.h"
struct TestMsg : public mini_ros::Message {
  std::uint32_t data;
  virtual ~TestMsg() {
    // std::cout << "~TestMsg, data:" << data << std::endl;
    print("~TestMsg, data:", data);
  }
};
#endif
