#ifndef TEST_MODULE_H_
#define TEST_MODULE_H_

#include "mini_ros.h"
#include "test_srv.h"

class TestModule : public mini_ros::Module
{

  bool test_srv(std::shared_ptr<TestSrv> srv)
  {
    srv->resp.data = srv->req.data + 100;
    return true;
  }

  void run() override
  {
    mini_ros::ModuleHandler& mh = getModuleHandler();
    mini_ros::ServiceServer server  = mh.advertiseService<TestSrv>("test_srv2",
      std::bind(&TestModule::test_srv, this, std::placeholders::_1));
  }
};

#endif
