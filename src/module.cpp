#include "module.h"

namespace mini_ros {

  Module::Module()
  {
    _t = std::thread([this]()
    {
      handle = new ModuleHandle();
      onInit();
      handle->spin();
      onExit();
      cv.notify_one();
    });
  }

  void Module::wait()
  {
    std::unique_lock <std::mutex> lck(mtx);
    cv.wait(lck);
  }

  void Module::sleep(uint32_t ms)
  {
    assert(std::this_thread::get_id() == _t.get_id());
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  void Module::stop()
  {
    onStopped();
    handle->stop();
  }

  void Module::onExit() {}
  void Module::onStopped() {}

  Module::~Module()
  {
    if (handle != nullptr)
    {
      delete[] handle;
    }
  }

  ModuleHandle& Module::getModuleHandle()
  {
    return *handle;
  }

} /* mini_ros */
