#ifndef SAMPLE_SRV_H_
#define SAMPLE_SRV_H_

#include <mini_ros/service.h>

struct SampleSrv : public mini_ros::Service
{
  std::uint32_t req;
  std::uint32_t resp;
};

#endif
