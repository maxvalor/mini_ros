#ifndef TOOL_H_
#define TOOL_H_
#include <iostream>
#include <string>
#include <mutex>

static std::mutex mtx;

void print(std::string str, std::uint32_t data) {
  mtx.lock();
  std::cout << str << data << std::endl;
  mtx.unlock();
}

#endif
