#include <mini_ros/mini_ros.h>
#include "server_module.h"
#include "client_module.h"


int main()
{
  mini_ros::init();
  ServerModule sm;
  ClientModule cm;
  mini_ros::hold(&sm, &cm);
  return 0;
}
