#include "mini_ros.h"
#include "test_msg.h"
#include "test_msg_2.h"
#include <thread>
#include <iostream>
#include <unistd.h>
#include "tool.h"

static int c;
mini_ros::Subscriber sub;

void on_msg(std::shared_ptr<TestMsg> msg) {
  //std::cout << "1:received " << msg->data << std::endl;
  print("1:received ", msg->data[0]);
}

void on_msg2(std::shared_ptr<TestMsg2> msg) {
  // std::cout << "2:received " << msg->data << std::endl;
  print("2:received ", msg->data);
}

void on_msg3(std::shared_ptr<TestMsg> msg) {
  // std::cout << "3:received " << msg->data << std::endl;
  print("3:received ", msg->data[0]);
  if (msg->data[0] == 5) {
    sub.shutdown();
  }
}

int main() {
  mini_ros::ModuleHandler& mh = mini_ros::ModuleHandler::instance();
  mh.subscribe<TestMsg>("test", on_msg);
  mh.subscribe<TestMsg2>("test2", on_msg2);
  sub = mh.subscribe<TestMsg>("test", on_msg3);
  std::thread t([&mh](){
    int i = 0;
    mini_ros::Publisher pub = mh.advertise<TestMsg>("test");
    mini_ros::Publisher pub2 = mh.advertise<TestMsg2>("test2");
    while (++i < 10) {
#if 0
      TestMsg msg;
      msg.data = new std::uint32_t[1];
      msg.data[0] = i;
      std::cout << "publish " << msg.data[0] << std::endl;
      //mh.publish<TestMsg>("test", msg);
      pub.publish(msg);
#endif
#if 1
      TestMsg *msg2 = new TestMsg();
      msg2->data = new std::uint32_t[1];
      msg2->data[0] = i + 100;
      std::shared_ptr<TestMsg> s_msg2(msg2);
      std::cout << "publish " << msg2->data[0] << std::endl;
      //mh.publish<TestMsg>("test", msg);
      pub.publish(s_msg2);
#endif
#if 0
      TestMsg2 msg3;
      msg3.data = i + 200;
      std::cout << "publish " << msg3.data << std::endl;
      //mh.publish<TestMsg>("test", msg);
      pub2.publish(msg3);
#endif
    }
  });
  t.join();
  usleep(200000);
  return 0;
}
