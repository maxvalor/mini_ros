#include "mini_ros.h"
#include "test_msg.h"
#include "test_msg_2.h"
#include "test_srv.h"
#include "test_module.h"

std::mutex print_mtx;

void on_msg(std::shared_ptr<TestMsg> msg) {
  print_mtx.lock();
  std::cout << std::this_thread::get_id() << " received "  << msg->data[0] << std::endl;
  print_mtx.unlock();
}

void on_msg1(std::shared_ptr<TestMsg> msg) {
  print_mtx.lock();
  std::cout << std::this_thread::get_id() << " received1 "  << msg->data[0] << std::endl;
  print_mtx.unlock();
}

void on_msg2(std::shared_ptr<TestMsg2> msg) {
  print_mtx.lock();
  std::cout << std::this_thread::get_id() << " received2 "  << msg->data << std::endl;
  print_mtx.unlock();
}

bool test_srv(std::shared_ptr<TestSrv> srv)
{
  srv->resp.data = srv->req.data + 100;
  return true;
}

int main() {
  mini_ros::init();
  mini_ros::ModuleHandler mh;
  TestModule tm;
  std::thread pub_t_1([](){
    mini_ros::ModuleHandler mh;
    mini_ros::ServiceServer server = mh.advertiseService<TestSrv>("test_srv", test_srv, false);
    //server.shutdown();
    //mini_ros::Subscriber sub = mh.subscribe<TestMsg>("test", on_msg);
    //mh.spin();
    int i = 10;
    mini_ros::Publisher pub = mh.advertise<TestMsg>("test");
    while (--i)
    {
      //std::this_thread::sleep_for(std::chrono::seconds(1));
      TestMsg *msg2 = new TestMsg();
      msg2->data = new std::uint32_t[1];
      msg2->data[0] = i;
      std::shared_ptr<TestMsg> s_msg2(msg2);
      print_mtx.lock();
      std::cout << "publish test:" << msg2->data[0] << std::endl;
      print_mtx.unlock();
      //mh.publish<TestMsg>("test", msg);
      pub.publish(s_msg2);
    }
    mh.spin();
  });

  std::thread pub_t_2([](){
    mini_ros::ModuleHandler mh;
    //mini_ros::Subscriber sub = mh.subscribe<TestMsg>("test", on_msg);
    //mh.spin();
    int i = 10;
    mini_ros::Publisher pub = mh.advertise<TestMsg2>("test2");
    mini_ros::ServiceClient client = mh.serviceClient<TestSrv>("test_srv", 10);
    mini_ros::ServiceClient client2 = mh.serviceClient<TestSrv>("test_srv", 10);
    while (--i)
    {
      TestMsg2 msg;
      std::shared_ptr<TestSrv> srv = std::make_shared<TestSrv>();
      srv->req.data = i;
      if (client.call(srv))
      {
        msg.data = srv->resp.data;
      }
      else
      {
        std::cout << "error " << std::endl;
      }
      std::cout << "publish test2:" << msg.data << std::endl;
      //mh.publish<TestMsg>("test", msg);
      pub.publish(msg);

      TestSrv srv2;

      srv2.req.data = i + 1000;
      if (client.call(srv2))
      {
        msg.data = srv2.resp.data;
      }
      else
      {
        std::cout << "error " << std::endl;
      }
      std::cout << "publish test2:2:" << msg.data << std::endl;
      //mh.publish<TestMsg>("test", msg);
      pub.publish(msg);
    }
    mh.spin();
  });

  std::thread t2([]{
    mini_ros::ModuleHandler mh;
    mini_ros::Subscriber sub = mh.subscribe<TestMsg>("test", on_msg);
    mh.spin();
  });
  mini_ros::Subscriber sub = mh.subscribe<TestMsg>("test", on_msg);
  mini_ros::Subscriber sub2 = mh.subscribe<TestMsg>("test", on_msg1);
  mini_ros::Subscriber sub3 = mh.subscribe<TestMsg2>("test2", on_msg2);
  //sub3.shutdown();
  //mini_ros::Subscriber sub4 = mh.subscribe<TestMsg2>("test2", on_msg2);


  pub_t_1.detach();
  pub_t_2.detach();
  t2.detach();
  mh.spin();
  return 0;
}
