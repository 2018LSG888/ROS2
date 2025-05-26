本仓库存放ROS2-humble下的编程代码，主要包括四种通信机制（话题、服务、参数、动作）以及 ROS2其他核心的东西

ctrl+/  单行注
ctrl+shift+A  多行注释
编译某个功能报：colcon build --packages-select pkg01_helloworld_cpp
运行某个程序：     ros2 run pkg02_helloworld_py helloworld
===============================================
ros2 创建工作空间    mkdir -p ws01_plumbing/src  
进入工作空间               cd ws01_plumbing
在工作空间中编译      colcon build
进入到src目录            cd src
创建新的功能包          ros2 pkg create --build-type ament_cmake base_interfaces_demo
------------------------------------------------
vscode中创建新的python功能包 和节点
ros2 pkg create py01_topic --build-type ament_python --dependencies rclpy std_msgs base_interfaces_demo --node-name demo01_talker_str_py

vscode中创建新的C++功能包 和节点
ros2 pkg create cpp01_topic --build-type ament_cmake --dependencies rclcpp std_msgs base_interfaces_demo --node-name demo01_talker_str

=======================================================
小乌龟测试案例
1.ros2 run turtlesim turtlesim_node

2.ros2 run turtlesim turtle_teleop_key

========================================================

colcon build  //编译整个工作空间所有功能包
. install/setup.bash
ros2 run pkg03_hellovscode_cpp hellovscode //运行程序

colcon build --packages-select pkg03_hellovscode_cpp //只编译pkg03功能包

----------------------------------------------------------
2.2.2话题通信-发布 C++
进入到ws01_plumbing工作空间
终端一：
1) colcon build
2) . install/setup.bash
3) colcon build --packages-select cpp01_topic
4) ros2 run cpp01_topic demo01_talker_str  //发布消息
终端二：
1) . install/setup.bash
2)  ros2 topic echo /chatter

-------------------

2.2.2话题通信-订阅 C++
1) colcon build --packages-select cpp01_topic

2) ros2 run cpp01_topic demo02_listener_str 

===========================================

2.2.3 话题通信-发布 python
终端一
1) colcon build --packages-select py01_topic
2)  . install/setup.bash
3) ros2 run py01_topic demo01_talker_str_py 
终端二  验证消息已经发出
1) . install/setup.bash
2) ros2 topic echo /chatter   
       或者 运行c++版的订阅程序  
   ros2 run cpp01_topic demo02_listener_str 


2.2.3 话题通信-订阅 python
1) colcon build --packages-select py01_topic
2) . install/setup.bash
3) ros2 run py01_topic demo02_listener_str_py 



两个终端都要先 . install/setup.bash

1) ros2 run cpp01_topic demo03_talker_stu 

2) ros2 run cpp01_topic demo04_listener_stu

============= 服务通信 =========

//创建cpp02_service 功能包 C++版本 ，依赖项和节点名字
ros2 pkg create cpp02_service --build-type ament_cmake --dependencies rclcpp base_interfaces_demo --node-name demo01_server

//创建py02_service 功能包 python版本， 依赖项和节点名字
ros2 pkg create py02_service --build-type ament_python --dependencies rclpy base_interfaces_demo --node-name demo01_server_py

