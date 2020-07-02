#include "module.h"

namespace mini_ros {

  Module::Module()
  {
    _t = std::thread([this]()
    {
      handler = new ModuleHandler();
      onInit();
      handler->spin();
      onExit();
    });
  }

  void Module::wait()
  {
    _t.join();
  }

  void Module::sleep(uint32_t ms)
  {
    assert(std::this_thread::get_id() == _t.get_id());
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  void Module::stop()
  {
    handler->stop();
  }

  void Module::onExit() {}

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
