#ifndef TEST_MSG_H_
#define TEST_MSG_H_
#include "message.h"
struct TestMsg : public mini_ros::Message {
  std::uint32_t* data;
  size_t len;

  TestMsg() : data(nullptr) {}

  TestMsg(TestMsg&& t) {
    data = t.data;
    t.data = nullptr;
    len = t.len;
  }

  virtual ~TestMsg() {
    // std::cout << "~TestMsg, data:" << data << std::endl;
    if (data != nullptr) {
      //print("~TestMsg, data:", data[0]);
      delete[] data;
    }
    else {
      //print("~TestMsg, no data.", 0);
    }
  }
};
#endif
