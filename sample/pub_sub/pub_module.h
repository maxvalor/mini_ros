#ifndef PUB_MODULE_H_
#define PUB_MODULE_H_

#include <mini_ros/module.h>
#include "sample_msg.h"

class PubModule : public mini_ros::Module {
private:
  /* data */

public:
  PubModule () {}
  virtual ~PubModule () {}

  void onInit() override
  {
    mini_ros::Publisher pub = getModuleHandler().advertise<SampleMsg>("sample_topic");
    int i = 10;
    while (--i)
    {
      SampleMsg msg;
      msg.data = new std::uint32_t[1];
      msg.data[0] = i;
      msg.len = 1;
      std::cout << "publish by object, data:" << msg.data[0] << std::endl;
      pub.publish(msg);

      std::shared_ptr<SampleMsg> spMsg(new SampleMsg());
      spMsg->data = new std::uint32_t[1];
      spMsg->data[0] = i;
      spMsg->len = 1;
      std::cout << "publish by shared pointer, data:" << spMsg->data[0] << std::endl;
      pub.publish(spMsg);

      sleep(1);
    }
  }
};

#endif
