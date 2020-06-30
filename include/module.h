#ifndef MODULE_H_
#define MODULE_H_

#include <thread>

namespace mini_ros {

class Module {
private:
  std::thread _t;

public:
  Module () {
    _t = std::thread([this]()
    {
      run();
    });
  }
  virtual ~Module ()
  {
    _t.join();
  }

  virtual void run() = 0;
};

} /* mini_ros */

#endif
