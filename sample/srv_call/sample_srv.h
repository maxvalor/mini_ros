#ifndef SAMPLE_SRV_H_
#define SAMPLE_SRV_H_

#include <mini_ros/service.h>

struct SampleSrv : public mini_ros::Service
{
<<<<<<< HEAD
  struct Request : public mini_ros::Service::Request
  {
    std::uint32_t data;
  } req;

  struct Response : public mini_ros::Service::Response
  {
    std::uint32_t data;
  } resp;
=======
  std::uint32_t req;
  std::uint32_t resp;
>>>>>>> dev
};

#endif
