#ifndef CLIENT_MODULE_H_
#define CLIENT_MODULE_H_

#include <mini_ros/module.h>
#include <iostream>
#include "sample_srv.h"

class ClientModule : public mini_ros::Module {
private:
  /* data */

public:
  ClientModule () {}
  virtual ~ClientModule () {}

  void onInit() override
  {
    mini_ros::ServiceClient client =
      getModuleHandle().serviceClient<SampleSrv>("sample_service");

      int i = 10;
      while (--i)
      {
        auto pSrv = std::make_shared<SampleSrv>();
        pSrv->req= i;
        if (client.call(pSrv))
        {
          std::cout << "call service by shared pointer, service return:" << pSrv->resp<< std::endl;
        }
        else
        {
          std::cout << "error " << std::endl;
        }
      }
  }
};

#endif
