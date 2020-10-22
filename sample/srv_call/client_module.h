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
        SampleSrv srv;
<<<<<<< HEAD
        srv.req.data = i;
        if (client.call(srv))
        {
          std::cout << "call service by object, service return:" << srv.resp.data << std::endl;
=======
        srv.req = i;
        if (client.call(srv))
        {
          std::cout << "call service by object, service return:" << srv.resp<< std::endl;
>>>>>>> dev
        }
        else
        {
          std::cout << "error " << std::endl;
        }

        std::shared_ptr<SampleSrv> pSrv = std::make_shared<SampleSrv>();
<<<<<<< HEAD
        pSrv->req.data = i;
        if (client.call(pSrv))
        {
          std::cout << "call service by shared pointer, service return:" << pSrv->resp.data << std::endl;
=======
        pSrv->req= i;
        if (client.call(pSrv))
        {
          std::cout << "call service by shared pointer, service return:" << pSrv->resp<< std::endl;
>>>>>>> dev
        }
        else
        {
          std::cout << "error " << std::endl;
        }
      }
  }
};

#endif
