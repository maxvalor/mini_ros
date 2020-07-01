#include "module.h"

namespace mini_ros {

  Module::Module()
  {
    _t = std::thread([this]()
    {
      handler = new ModuleHandler();
      run();
      handler->spin();
    });
  }

  Module::~Module()
  {
    _t.join();

    if (handler != nullptr)
    {
      delete[] handler;
    }
  }

  ModuleHandler& Module::getModuleHandler()
  {
    return *handler;
  }

} /* mini_ros */
