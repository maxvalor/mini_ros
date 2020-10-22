#ifndef SERVER_MODULE_H_
#define SERVER_MODULE_H_

#include <mini_ros/module.h>
#include <iostream>
#include "sample_srv.h"

class ServerModule : public mini_ros::Module {
private:
  /* data */

public:
  ServerModule () {}
  virtual ~ServerModule () {}

  bool onCalled(std::shared_ptr<SampleSrv> srv)
  {
<<<<<<< HEAD
    std::cout << "receive request:" << srv->req.data << std::endl;
    srv->resp.data = srv->req.data + 100;
=======
    std::cout << "receive request:" << srv->req << std::endl;
    srv->resp = srv->req + 100;
>>>>>>> dev
    return true;
  }

  void onInit() override
  {
    mini_ros::ServiceServer server =
      getModuleHandle().advertiseService<SampleSrv>("sample_service",
        std::bind(&ServerModule::onCalled, this, std::placeholders::_1));
  }
};

#endif
