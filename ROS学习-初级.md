## 基本概念
1. **ROS文件系统**
ROS文件系统是ROS源代码在硬盘上的组织形式，包含packages、源代码等等
2. **ROS功能包：package** 
功能包是ROS的基本单元(一个文件夹)，包含源代码、库文件等等，它可以包括多个节点(从代码的角度来看就是生成多个可执行程序)，至少要包含两部分：package.xml和CMakeLists.txt
3. **ROS节点：node** 
节点就是一个可执行文件/可运行的程序/进程
4. **ROS命名空间：namespace** 
和C++等编程语言里的命名空间功能类似，避免重名的冲突问题
5. **ROS文件系统的组织结构** 
```
WorkSpace --- 自定义的工作空间

    |--- build:编译空间，用于存放CMake和catkin的缓存信息、配置信息和其他中间文件。

    |--- devel:开发空间，用于存放编译后生成的目标文件，包括头文件(比如include下通过msg或者service生成的头文件)、share文件夹下的动态&静态链接库、lib文件夹下的可执行文件等，

    |--- src: 源码，存放自己的数据包等
```
![ROS文件系统组织结构](http://www.autolabor.com.cn/book/ROSTutorials/assets/%E6%96%87%E4%BB%B6%E7%B3%BB%E7%BB%9F.jpg)
我开始的时候就把通过`catkin_create_pkg`命令创建的软件包位置放错了，[ Error: no such package beginner_tutorials“](https://answers.ros.org/question/222344/rospack-error-no-such-package-beginner_tutorials/)
6. 

## ROS一些常用命令

### ROS shell命令
操作功能包的命令，都比较简单，基本是ros前缀+linux命令的形式，比如roscd、rosls、roscp之类的
*补充：* rosed = ros + ed(itor)，编辑功能包的文件，官网有讲可以修改默认的编辑器

### ROS 执行命令
1. **roscore：ros master + rosout + parameter server** 
ros是分布式架构，节点之间通信达时候需要主节点(命名节点)协调，执行roscore启动，主节点接收多种信息的注册，如节点的名称、话题和服务名称、消息类型、URI地址和端口号，同时还会启动rosout节点作为ros的标准输入输出，和一个参数服务器
2. **rosrun 功能包名称 节点名称** 
执行特定功能包下某个节点，自定义的功能包节点也可以不用rosrun执行，直接用./的形式也可以
3.  **roslaunch 功能包名称 launch文件(xx.launch)** 
roslaunch可以同时启动多个节点，比如一次性启动几十个节点，方便
4.  **rosclean** 
删除或查看ros日志大小
查看：roclean check

### ROS信息命令
1. **rosnode**
2. **rostopic**
3. **rosservice**
4. **rosparam** 
5. **其他：** rosmsg、rosbag、rossvr、rosversion等 

## ROS节点通信方式
### （一）topic话题
topic话题通信方式是通过发布订阅模式进行消息传递的，所以也是异步的通信方式，我开始以为是会使用master节点作为消息中转的，但并不是，发布节点和订阅节点之间都是直连的，给予**TCP/IP** 且**单向传输** 。
1. **通信流程：**
![[Drawing 2022-12-02 16.19.26.excalidraw | 同步到git上看不到该插件生成都图片，可以参考同目录下的同名png图片]]

2. **通信数据载体：msg** 
ROS 中通过 std_msgs 封装了一些原生的数据类，包括：
-  int8, int16, int32, int64 (或者无符号类型: uint*)
-  float32, float64
-  string
-  time, duration
-  other msg files
-  variable-length array[] and fixed-length array
-  header(*目前还没见到过这种数据类型，待后续补充*)
-  ……

但在实际开发中，肯定需要很多和项目相关的自定义数据消息，以官网上的那个订阅/发布例子位例子，就可以把直接通过`std_msgs::String msg;`创建msg的方式修改一下，引用自定义msg里的格式数据，就不贴图了，简单来说就是以下几步：
-  在msg目录下添加自定义的msg文件，比如`Name.msg`
-  在`Name.msg`里添加数据格式，比如`string name`
-  把这个msg文件添加进CmakeLists.txt里
`` add_message_files(``
    ``FILES``
    ``Num.msg``
   `` Name.msg``
`` )``
-  在`talker.cpp`里引入头文件，这个是编译之后生成的，在devel/include里
``#include "beginner_tutorials/Name.h``
-  发布方式:
``ros::Publisher chatter_pub = n.advertise<beginner_tutorials::Name>("chatter", 1000);``
-  发布消息：
``msg.name = ss.str();``
-  listener.cpp类似
-  catkin_make编译

3. **优缺点：** 
-  **优点** ：适合连续的数据流传输，单向，不需要消息确认，高效
-  **缺点** ：单向传输，没有确认，可能丢失数据或者消息过载
-  **应用场景** ：激光雷达收集信息->导航模块处理信息->底盘……
### （二）service服务

### （三）parameter服务器

### （四）action库

## 参考链接

[古月居ROS命令总结](https://www.guyuehome.com/34842)

