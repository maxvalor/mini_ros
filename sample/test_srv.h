#ifndef TEST_SRV_H_
#define TEST_SRV_H_

#include "service.h"

struct TestSrv : public mini_ros::Service
{
  struct Request : public mini_ros::Service::Request
  {
    std::uint32_t data;
  } req;

  struct Response : public mini_ros::Service::Response
  {
    std::uint32_t data;
  } resp;
};

#endif
