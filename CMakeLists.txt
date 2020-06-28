cmake_minimum_required(VERSION 3.0)

project(mini_ros CXX)

add_definitions(-std=c++11)

include_directories(
  sample
  include
)

add_executable(mini_ros_sample
  sample/main.cpp
)

target_link_libraries(mini_ros_sample
  pthread
)
