cmake_minimum_required(VERSION 3.0)

project(mini_ros CXX)

add_definitions(-std=c++11)

include_directories(
  sample
  include
)

add_library(miniros SHARED
  src/core.cpp
  src/message_queue.cpp
  src/module.cpp
  src/thread_handler.cpp
)

add_executable(mini_ros_test
  test/main.cpp
)

target_link_libraries(mini_ros_test
  pthread
  miniros
)
