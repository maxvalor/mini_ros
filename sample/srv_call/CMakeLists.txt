cmake_minimum_required(VERSION 3.0)

project(sample_sc CXX)

add_definitions(-std=c++11)

include_directories(
  include
)

add_executable(sample_sc ./main.cpp)

set(SAMPLE_DEPENDS miniros)
target_link_libraries(sample_sc ${SAMPLE_DEPENDS})
