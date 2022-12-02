## ROS简介
### （一） 什么是ROS？
ROS，Robot Operating System，直译为机器人操作系统，但其实并不是真正的操作系统，本质还要依托于Linux内核实现一系列操作，可以理解为是一个中间件，但其功能类似于OS，OS对计算机底层各种硬件(声卡、网卡)进行了封装，ROS也是对机器人的硬件进行了抽象封装，包括不同的机器人、不同的传感器等，同时兼具底层设备控制，常用函数的实现，进程间消息传递，以及包管理等功能。
![传统OS和ROS对比](http://mmbiz.qpic.cn/mmbiz_png/0xXXr0YWD8Cg7tibW4LL6Pqjf81t41zn4FkrIuVhicJrKUicfDO9cr3OAyFBWEcr9ldg2dyagQl1jMicGWfbfQrZnw/640?wx_fmt=png&wxfrom=5&wx_lazy=1&wx_co=1)

### （二）ROS包括
1. 框架：ROS采用了分布式架构，用节点表示进程，兼具进程管理，可以同时运行多个节点，并组织节点间进行通信。
2. 功能：运动控制、运动规划、视觉、建图等
3. 工具：仿真、数据可视化、图形界面、数据记录等等，比如rqt
4. 社区：文档、软件包管理等
### （三）优点
1. 有一系列开源工具：有rqt等一系列图形化调试工具
2. 包含一系列开源算法：集成了OpenCV、Player等算法
3. 跨平台：ROS可以在不同的计算机平台、不同机器人、不同语言上使用，且定义了模块化软件通信方式，不同的应用程序作为节点通信，采用Topic、Service、Paragram参数服务器、Action库等通信方式
4. 其他：可扩展性好、代码复用率高、松耦合等等

## ROS环境安装

参照[wiki上的ROS教程](http://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment)安装即可，在安装过程中可能碰到一些问题，多数在[ROS官网 Q&A](https://answers.ros.org/questions/)中都可以找到。

### 安装过程中遇到的问题

1. 基本上这篇[博文](https://blog.csdn.net/hxj0323/article/details/121215992)都有总结
2. 安装成功后启用`roscore`测试时失败，提示找不到`command`[解决方案](https://www.codeleading.com/article/42014334083/)

## 参考链接

[# 机器人操作系统ROS | 简介篇](https://mp.weixin.qq.com/s?__biz=MzA5MDE2MjQ0OQ==&mid=2652786427&idx=1&sn=ac4b9a890fec3d68773c6cb65ed86946&mpshare=1&scene=1&srcid=0326mGy0nXBqFqBWxQCeNv6B&pass_ticket=1eHBdZLk6YjR3YAN0wCTt5ZXh4HtyqvVaLglwMh4ZRmOyALdqcjLhuOqe%2BBWkmJm#rd)
[ROS概述](http://www.autolabor.com.cn/book/ROSTutorials/chapter1/11-rosjian-jie-yu-an-zhuang/111rosgai-nian.html)

[wiki上的ROS教程](http://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
[安装过程中问题总结](https://blog.csdn.net/hxj0323/article/details/121215992)
[安装成功后但找不到roscore命令解决方案](https://www.codeleading.com/article/42014334083/)
