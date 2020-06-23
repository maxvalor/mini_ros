#include "mini_ros.h"
#include "test_msg.h"
#include <thread>
#include <iostream>
#include "tool.h"

static int c;
mini_ros::Subscriber<TestMsg> sub;

void on_msg(std::shared_ptr<TestMsg> msg) {
  //std::cout << "1:received " << msg->data << std::endl;
  print("1:received ", msg->data);
}

void on_msg2(std::shared_ptr<TestMsg> msg) {
  // std::cout << "2:received " << msg->data << std::endl;
  print("2:received ", msg->data);
}

void on_msg3(std::shared_ptr<TestMsg> msg) {
  // std::cout << "3:received " << msg->data << std::endl;
  print("3:received ", msg->data);
  if (msg->data == 5) {
    sub.shutdown();
  }
}

int main() {
  mini_ros::ModuleHandler& mh = mini_ros::ModuleHandler::instance();
  mh.subscribe<TestMsg>("test", on_msg);
  mh.subscribe<TestMsg>("test", on_msg2);
  sub = mh.subscribe<TestMsg>("test", on_msg3);
  std::thread t([&mh](){
    int i = 10;
    mini_ros::Publisher<TestMsg> pub = mh.advertise<TestMsg>("test");
    while (i--) {
      TestMsg msg;
      msg.data = i;
      std::cout << "publish " << msg.data << std::endl;
      //mh.publish<TestMsg>("test", msg);
      pub.publish(msg);
    }
  });
  t.join();
  return 0;
}
