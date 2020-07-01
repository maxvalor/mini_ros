#ifndef SAMPLE_MSG_H_
#define SAMPLE_MSG_H_

#include "mini_ros/mini_ros.h"
#include "sample_msg.h"

struct SampleMsg : public mini_ros::Message {
  std::uint32_t* data;
  size_t len;

  SampleMsg() : data(nullptr) {}

  SampleMsg(SampleMsg&& t) {
    data = t.data;
    t.data = nullptr;
    len = t.len;
  }

  virtual ~SampleMsg() {
    // std::cout << "~SampleMsg, data:" << data << std::endl;
    if (data != nullptr) {
      //print("~SampleMsg, data:", data[0]);
      delete[] data;
    }
    else {
      //print("~SampleMsg, no data.", 0);
    }
  }
};
#endif
