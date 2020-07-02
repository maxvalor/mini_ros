#include <mini_ros/mini_ros.h>
#include "sample_msg.h"

void onMsg(std::shared_ptr<SampleMsg> msg)
{
  std::cout << "receive data:" << msg->data[0] << std::endl;
}

int main()
{
  mini_ros::init();
  std::thread pub_t([]
  {
    mini_ros::ThreadHandler th;
    mini_ros::Publisher pub = th.advertise<SampleMsg>("sample_topic");
    int i = 10;
    while (--i)
    {
      SampleMsg msg;
      msg.data = new std::uint32_t[1];
      msg.data[0] = i;
      msg.len = 1;
      std::cout << "publish by object, data:" << msg.data[0] << std::endl;
      pub.publish(msg);

      std::shared_ptr<SampleMsg> spMsg(new SampleMsg());
      spMsg->data = new std::uint32_t[1];
      spMsg->data[0] = i;
      spMsg->len = 1;
      std::cout << "publish by shared pointer, data:" << spMsg->data[0] << std::endl;
      pub.publish(spMsg);

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  std::thread sub_t([]
  {
    mini_ros::ThreadHandler th;
    mini_ros::Subscriber sub = th.subscribe<SampleMsg>("sample_topic", onMsg);
    th.spin();
  });

  pub_t.join();
  sub_t.join();

  return 0;
}
