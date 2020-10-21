cmake_minimum_required(VERSION 3.0)

project(sample_th CXX)

add_definitions(-std=c++11)

include_directories(
  include
)

add_executable(sample_th ./main.cpp)

set(SAMPLE_DEPENDS miniros pthread)
target_link_libraries(sample_th ${SAMPLE_DEPENDS})
