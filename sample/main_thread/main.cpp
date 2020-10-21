#include <mini_ros/mini_ros.h>
#include "ros_module.h"


int main()
{
  mini_ros::init();
  ROSModule rm;
  mini_ros::hold(&rm);
  return 0;
}
