#include "mini_ros.h"
#include "test_msg.h"

void on_msg(std::shared_ptr<TestMsg> msg) {
  std::cout << std::this_thread::get_id() << " received "  << msg->data[0] << std::endl;
}

int main() {
  mini_ros::ModuleHandler mh;
  std::thread t([](){
    mini_ros::ModuleHandler mh;
    //mini_ros::Subscriber sub = mh.subscribe<TestMsg>("test", on_msg);
    //mh.spin();
    int i = 10;
    mini_ros::Publisher pub = mh.advertise<TestMsg>("test");
    while (--i)
    {
      TestMsg *msg2 = new TestMsg();
      msg2->data = new std::uint32_t[1];
      msg2->data[0] = i + 100;
      std::shared_ptr<TestMsg> s_msg2(msg2);
      std::cout << "publish " << msg2->data[0] << std::endl;
      //mh.publish<TestMsg>("test", msg);
      pub.publish(s_msg2);
    }
  });
  std::thread t2([]{
    mini_ros::ModuleHandler mh;
    mini_ros::Subscriber sub = mh.subscribe<TestMsg>("test", on_msg);
    mh.spin();
  });
  mini_ros::Subscriber sub = mh.subscribe<TestMsg>("test", on_msg);
  t.detach();
  t2.detach();
  mh.spin();
  return 0;
}
