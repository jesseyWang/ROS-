#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/Name.h"

#include <string>

int main(int argc, char **argv)
{
  /**
   * ros::init函数       初始化节点 4个参数 最后一个是可选项
   * argc               参数个数
   * argv               参数列表
   * name               参数名，有唯一性的要求，而且不能带命名空间的
   * options            可选
   */
  ros::init(argc, argv, "talker");

  // ros::NodeHandle    创建一个节点句柄，向master注册节点信息
  ros::NodeHandle n;

  /**
    * ros::Publisher     创建一个Publish可发布对象，msg类型是beginner_tutorials::Name的名为chatter的话题，
    * msg队列长度为1000，如果发布方数据太多超过这个限制就会丢掉队头的数据
    */
  ros::Publisher pub = n.advertise<beginner_tutorials::Name>("chatter", 1000);

  // ros::Rate            时钟，按照一定频率执行循环 hz为单位 这里是每秒执行一次周期
  ros::Rate rate(1);

  int count = 0;
  while (ros::ok())
  {
    // 自定义的msg
    beginner_tutorials::Name msg;

    std::string ss = "hello world! " + std::to_string(count);
    msg.name = ss;

    ROS_INFO("%s", msg.name.c_str());

    pub.publish(msg);
    
    /**
    * ros::spinOnce() 消息回调处理函数 在消息订阅中只有调用回调处理函数才会取出或发出数据
    * 一般可以执行在函数的任意位置，这个函数只会调用一次，主程序执行完这个函数之后还会执行之后的代码
    */
    ros::spinOnce();

    // ros::sleep         用sleep度过后面的时间
    rate.sleep();

    ++count;
  }

  return 0;
}
