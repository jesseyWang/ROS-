#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/Name.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const beginner_tutorials::Name::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->name.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  /**
   * The subscribe() 订阅函数
   * topic           订阅的话题
   * size            接收队列的大小
   * callback        回调函数
   * object          第四个参数，类对象成员函数的调用时，指定对象的类或者传this上下文指针
   */
  // TODO：待改成类的实现方式
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
    * ros::spin() 消息回调处理函数 在消息订阅中只有调用回调处理函数才会取出或发出数据
    * 一般可以在程序最后，这个函数会一直调用，直到ctr c或roscore退出
    */
  ros::spin();

  return 0;
}