[TOC]
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
topic话题通信方式是通过发布订阅模式进行消息传递的，所以也是异步的通信方式，我开始以为是会使用master节点作为消息中转的，但并不是，发布节点和订阅节点之间都是直连的，基于**TCP/IP** 且**单向传输**，订阅节点等待发布节点的数据推送，本身不会有请求的动作。

1. **通信流程：**
![[topic通信流程.png | 同步到git上看不到该插件生成的图片，可以参考同目录下的同名png图片]]

2. **通信数据载体：msg** 
ROS 中通过 std_msgs 封装了一些原生的数据类，作为topic通信的内容，catlin_make编译后会生成对应的头文件，类似于protobuffer，可以看作一个结构体或类，包括：
-  int8, int16, int32, int64 (或者无符号类型: uint*)
-  float32, float64
-  string
-  time, duration
-  other msg files
-  variable-length array[] and fixed-length array
-  header(*目前还没见到过这种数据类型，待后续补充*)
-  ……

但在实际开发中，肯定需要很多和项目相关的自定义数据消息，以官网上的那个订阅/发布例子为例，就可以把直接通过`std_msgs::String msg;`创建msg的方式修改一下，引用自定义msg里的格式数据，就不贴图了，简单来说就是以下几步：
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

**注意：** 如果是在vscode下需要配置`c_cpp_properies.json 文件`的工作空间，加上`xx/devel/include/**`头文件路径

3. **优缺点：** 
-  优点：适合连续的数据流传输，单向，不需要消息确认，高效
-  缺点：单向传输，没有确认，可能丢失数据或者消息过载
-  应用场景 ：激光雷达收集信息->导航模块处理信息->底盘……

### （二）service服务
service通信是一种同步阻塞的通信方式，节点之间通过请求-应答的方式通信，这也是和topic通信的最大不同，其他类似，服务器节点和客户端节点也是先需要和ROS Master注册自身信息，等待master匹配相同服务的客户端和服务器之后建连，但因为是请求应答的形式，客户端请求之前服务器首先要是已启动到状态。

1. **通信流程** 
![[service通信流程.png | 同步到git上看不到该插件生成的图片，可以参考同目录下的同名png图片]]

2. **通信数据载体：svr**
ROS 中service使用svr文件作为通信内容，和msg类似，使用的都是和msg一致的类型，不过svr文件包含两部分，服务器和客户端的数据，用`---`上下分隔，catlin_make编译后也会生成对应的头文件。

3. **优缺点**
-   优点：请求-应答模式，客户端知道是否能调用成功，且有服务反馈
-   缺点：建立通信慢，有等待延时
4. 

### （三）parameter服务器
参数服务器通信和前两种很大的不同就是它需要ROS Master的协作，ROS Master可以作为一个公共的容器存储参数，也可以说是参数服务器，性能一般，适合简单的数据存储。

1. **通信流程**
![[parameter通信流程.png | 同步到git上看不到该插件生成的图片，可以参考同目录下的同名png图片]]

2.  **参数数据类型**
-   32-bit integers
-   booleans
-   strings
-   doubles
-   iso8601 dates
-   lists
-   base64-encoded binary data
-   字典

*补充：像C++里设置参数的话还可以设置vector、map之类的参数* 

3.  **优缺点** 
-   优点：简单
-   缺点：性能一般
4.  ROS API：`ros::NodeHandle` 和`ros::param`
5. 

### （四）action库
回想一下前面几种通信方式的优缺点，对于topic通信，如果订阅节点想给发布节点一个反馈的话需要自己再发布一个话题，让发布节点订阅反馈话题，需要两个话题才能完成完整的反馈链；服务通信倒是有反馈机制，但客户端只有在服务器服务结束的时候才会有反馈，在服务器服务期间无法打断服务器的处理过程……所以action对二者折中了一下，**满足了**：
-   实时反馈通信状态和信息
-   有条件的打断通信联系
-   通信双方稳定存在，但不需要一致存在
-   ……

action通信的数据载体是`.action`文件，但其本质也是通过**topic话题**通信，只不过`actionlib`功能包对msg进行了封装。
在服务器和客户端之间通过goal, cancel, status, feedback, result五个主题实现Action机制。
-   goal：客户端请求
-   cancle：客户端取消请求操作，分两种：真正的断连和等待意外事件处理的暂时性中止通信，等意外事件处理完继续通信
-   status： 通信的状态，取值固定，**不需要用户自己设置**，包括`PENDING=0 ACTIVE=1  PREEMPTED=2  SUCCEEDED=3  ABORTED=4  REJECTED=5  PREEMPTING=6  RECALLING=7  RECALLED=8  LOST=9`
-   feedback： 服务器对客户端的实时反馈，比如正在运动的小车的姿态信息等
-   result： 服务器对客户端的最终反馈
1. **通信流程**
注册节点的过程和topic话题通信类似
![[action通信流程.png]]

2. **通信数据载体**：action
需要用户自己设置的有goal、result、feedback三个主题，定义在一个`.action`文件里，由`---`分隔，同时像使用msg和svr那样设置CMakeLists.txt、package.xml中添加`actionlib`和`actionlib_msgs`依赖

3. **补充**
action通信的客户端和服务器只考虑goal的处理，也只会追踪一个goal，其他的goal(如果有的话)相当于被抢占。action通信里的客户端和服务器叫`SimpleActionClient`和`SimpleActionServer`，`SimpleActionServer`会为每个goal创建一个有限状态机来管理它的状态。

4. **优缺点**
-   优点：能够监控长时间的进程，可以进行实时反馈，有条件的打断通信等
-   缺点：实现起来相对复杂
6. 

## 参考链接

[古月居ROS命令总结](https://www.guyuehome.com/34842)  
[ROS 通信方式](http://www.autolabor.com.cn/book/ROSTutorials/di-2-zhang-ros-jia-gou-she-ji/22hua-ti-tong-xin.html)  
[# ROS之Action动作通信详解（分析+源码）](https://blog.csdn.net/weixin_45590473/article/details/122525228)  
[# ROS探索总结（三十二）——action](http://www.guyuehome.com/908)  
[ROS actionlib学习](https://www.cnblogs.com/21207-iHome/p/8304658.html)
