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

  void Module::wait()
  {
    _t.join();
  }

  void Module::sleep(uint32_t ms)
  {
    if (std::this_thread::get_id() != _t.get_id())
    {
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  Module::~Module()
  {
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
