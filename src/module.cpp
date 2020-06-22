#include "module.h"

void Module::load() {
  std::thread t(&Module::onload, this);
  __module_manager.registerModule(this);
}

void ModuleManager::registerModule(Module* m)
{
  modules.push_back(m);
}
void ModuleManager::waitAllModule() {
  for (auto m : modules) {
    m->__t.join();
  }
}
