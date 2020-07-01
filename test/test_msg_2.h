#ifndef TEST_MSG_2_H_
#define TEST_MSG_2_H_
#include "message.h"
struct TestMsg2 : public mini_ros::Message {
  std::uint32_t data;

  virtual ~TestMsg2() {
    //print("~TestMsg2, data:", data);
  }
};
#endif
