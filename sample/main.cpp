#include "mini_ros.h"
#include "test_msg.h"
#include <thread>
#include <iostream>

void on_msg(std::shared_ptr<TestMsg> msg) {
  std::cout << "1:received " << msg->data << std::endl;
}

void on_msg2(std::shared_ptr<TestMsg> msg) {
  std::cout << "2:received " << msg->data << std::endl;
}

void on_msg3(std::shared_ptr<TestMsg> msg) {
  std::cout << "3:received " << msg->data << std::endl;
}

int main() {
  mini_ros::ModuleHandler& mh = mini_ros::ModuleHandler::instance();
  mh.subscribe<TestMsg>("test", on_msg);
  mh.subscribe<TestMsg>("test", on_msg2);
  mh.subscribe<TestMsg>("test", on_msg3);
  std::thread t([&mh](){
    int i = 200;
    while (i--) {
      TestMsg msg;
      msg.data = i;
      std::cout << "publish " << msg.data << std::endl;
      mh.publish<TestMsg>("test", msg);
    }
  });
  t.join();
  return 0;
}
