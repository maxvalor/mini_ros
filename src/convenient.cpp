#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <stdarg.h>
#include "module.h"

namespace mini_ros {

void init()
{
  Core::instance();
}

void stop(int signo)
{
    _exit(0);
}

bool hold(size_t count, ...)
{
  va_list ap;
  va_start(ap, count);

  signal(SIGINT, stop);

  for (int i = 0; i < count; ++i) {
    Module *pModule = va_arg(ap, Module*);
    pModule->wait();
  }

  va_end(ap);

  return true;
}

} /* mini_ros */
