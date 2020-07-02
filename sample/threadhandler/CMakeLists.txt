cmake_minimum_required(VERSION 3.0)

project(mini_ros_sample_threadhandler CXX)

add_definitions(-std=c++11)

include_directories(
  ../../include/
)
add_executable(sample_th
  main.cpp
)

set(MINI_ROS
  ../../../build/libminiros.so
)

target_link_libraries(sample_th
  #${MINI_ROS}
  /home/ubuntu/workspace/mini_ros/build/libminiros.so
  pthread
)
