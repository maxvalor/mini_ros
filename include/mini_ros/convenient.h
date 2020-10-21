#ifndef CONVENIENT_H_
#define CONVENIENT_H_

#include "main_thread.h"

namespace mini_ros {

void init(int argc = 0, char** argv = nullptr);


void __hold(size_t count, ...);

template <typename ...T>
void hold(T... modules)
{
  __hold(sizeof...(modules), modules...);
}

void main_thread_wait(MainThreadEvent& event);
void main_thread_send(MainThreadEvent& event);

struct argc
{
  int operator()();
};

struct argv
{
  char** operator()();
};

} /* mini_ros */

#endif
