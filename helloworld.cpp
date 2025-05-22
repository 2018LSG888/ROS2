/*
  需求：在终端输出文本 hello world。
 流程：
      1.包含头文件；
      2.初始化ROS2客户端；
      3.创建节点指针；
      4.输出日志；
      5.释放资源。

*/

#include "rclcpp/rclcpp.hpp"


// 方式1(不推荐)
/* int main(int argc, char **argv){
	
	//2.初始化ROS2客户端；
        rclcpp::init(argc,argv);
        //3.创建节点指针；
        auto node = rclcpp::Node::make_shared("helloworld_node_cpp");
        //4.输出日志；
        RCLCPP_INFO(node->get_logger(),"hello world!");
        //5.释放资源。
	rclcpp::shutdown();
	return 0;
}
 */

// 方式2(推荐)
// 自定义类继承 Node
class MyNode: public rclcpp::Node{
public:
        MyNode():Node("hello_node_cpp"){
                RCLCPP_INFO(this->get_logger(),"hello world!(继承的方式)");
        }

};
/*  
        问题：初始化与资源释放在程序中起什么作用？
        答：
                1.前提:构建的程序可能由若干步骤或阶段组成；
                  初始化-->节点对象-->日志输出-->数据的发布--->数据订阅-->...-->资源释放。
                2.不同步骤或阶段之间涉及到数据的传递。
                3.怎么实现数据的传递呢？
                  使用使用Context（上下文）对象，这是一个容器，可以存储数据，也可以从中读取数据。
                4.初始化其实就是要创建Context对象，资源释放就是要销毁Context对象。

*/

int main(int argc, char const *argv[])
{
        // 初始化
        rclcpp::init(argc,argv);        
        // 实例化自定义类
        auto node = std::make_shared<MyNode>();
        // .....

        // 资源释放
        rclcpp::shutdown();

        return 0;
}















