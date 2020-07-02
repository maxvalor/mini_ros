cmake_minimum_required(VERSION 3.0)

project(mini_ros_sample_srv_call CXX)

add_definitions(-std=c++11)

include_directories(
  ../../include/
)
add_executable(sample_sc
  main.cpp
)

set(MINI_ROS
  ../../../build/libminiros.so
)

target_link_libraries(sample_sc
  #${MINI_ROS}
  /home/ubuntu/workspace/mini_ros/build/libminiros.so
  pthread
)
