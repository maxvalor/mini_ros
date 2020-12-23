#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdarg.h>
#include "module.h"
#include "convenient.h"

#include <iostream>

namespace mini_ros {

int __argc = 0;
char** __argv = nullptr;

int argc::operator()()
{
  return __argc;
}

char** argv::operator()()
{
  return __argv;
}

void stop(int signo)
{
  MainThreadProcessor::instance().stop();
}

void init(int argc, char** argv)
{
  __argc = argc;
  __argv = argv;
  Core::instance();
  MainThreadProcessor::instance();
  signal(SIGINT, stop);
}

void main_thread_wait(MainThreadEvent& event)
{
  MainThreadEventSP event_sp(new MainThreadEvent(event.get_f()));
  MainThreadProcessor::instance().waitEvent(event_sp);
}
void main_thread_send(MainThreadEvent& event)
{
  MainThreadEventSP event_sp(new MainThreadEvent(event.get_f()));
  MainThreadProcessor::instance().addEvent(event_sp);
}

void __hold(size_t count, ...)
{
  va_list ap;
  va_start(ap, count);

  MainThreadProcessor::instance().processEvent();

  for (size_t i = 0; i < count; ++i)
  {
    std::cout << count << std::endl;
    Module *pModule = va_arg(ap, Module*);
    pModule->stop();
    std::future<void> result = std::async([pModule](){
      pModule->wait();
    });
    try
    {
      std::chrono::system_clock::time_point two_seconds_passed
          = std::chrono::system_clock::now() + std::chrono::milliseconds(10);
      result.wait_until(two_seconds_passed);
    }
    catch (...) {}
  }

  va_end(ap);

  _exit(0);
}

} /* mini_ros */
