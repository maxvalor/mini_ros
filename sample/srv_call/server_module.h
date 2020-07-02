#ifndef SERVER_MODULE_H_
#define SERVER_MODULE_H_

#include "mini_ros/mini_ros.h"
#include "sample_srv.h"

class ServerModule : public mini_ros::Module {
private:
  /* data */

public:
  ServerModule () {}
  virtual ~ServerModule () {}

  bool onCalled(std::shared_ptr<SampleSrv> srv)
  {
    std::cout << "receive request:" << srv->req.data << std::endl;
    srv->resp.data = srv->req.data + 100;
    return true;
  }

  void onInit() override
  {
    mini_ros::ServiceServer server =
      getModuleHandler().advertiseService<SampleSrv>("sample_service",
        std::bind(&ServerModule::onCalled, this, std::placeholders::_1));
  }
};

#endif
