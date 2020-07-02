cmake_minimum_required(VERSION 3.0)

project(mini_ros CXX)

add_definitions(-std=c++11)

include_directories(
  include/mini_ros
)

add_library(miniros SHARED
  src/core.cpp
  src/message_queue.cpp
  src/module.cpp
  src/thread_handler.cpp
)

target_link_libraries(miniros
  pthread
)

include(cmake/sample.cmake)
