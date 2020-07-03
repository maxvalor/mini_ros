#ifndef SUB_MODULE_H_
#define SUB_MODULE_H_

#include <mini_ros/module.h>
#include <iostream>
#include "sample_msg.h"

class SubModule : public mini_ros::Module {
private:
  /* data */

public:
  SubModule () {}
  virtual ~SubModule () {}

  void onMsg(std::shared_ptr<SampleMsg> msg)
  {
    std::cout << "receive data:" << msg->data[0] << std::endl;
  }

  void onInit() override
  {
    mini_ros::Subscriber sub =
      getModuleHandler().subscribe<SampleMsg>("sample_topic",
        std::bind(&SubModule::onMsg, this, std::placeholders::_1));
  }
};

#endif
