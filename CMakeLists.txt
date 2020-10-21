cmake_minimum_required(VERSION 3.0)

project(mini_ros VERSION 1.0)

add_definitions(-std=c++11)

include_directories(
  include/mini_ros
)

add_library(miniros SHARED
  src/core.cpp
  src/main_thread.cpp
  src/module.cpp
  src/thread_handle.cpp
  src/convenient.cpp
)

target_link_libraries(miniros
  pthread
)

include(cmake/sample.cmake)

install(TARGETS miniros
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)
