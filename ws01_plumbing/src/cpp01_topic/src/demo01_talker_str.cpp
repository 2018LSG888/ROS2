/* 
  需求：以某个固定频率发送文本“hello world！”，文本后缀编号，每发布一条，编号+1。
  流程：
    1.包含头文件；
    2.初始化ROS2客户端；
    3.自定义节点类；
      3-1.创建消息发布方；
      3-2.创建定时器；
      3-3.组织并发布消息。
    4.调用spin函数，传入自定义类对象指针；
    5.释放资源；

 */
// 1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// 3.自定义节点类； 定义了 Talker 类，并继承自 rclcpp::Node
class Talker: public rclcpp::Node{
public:
  // 构造函数中调用基类 rclcpp::Node 的构造函数，并传递节点名 talker_node_cpp  , count(0)是初始化为0
  Talker():Node("talker_node_cpp"),count(0){
    RCLCPP_INFO(this->get_logger(),"发布节点创建！");
    // 3-1.创建消息发布方；
    /* 
      模板：被发布的消息类型；
      参数：
        1.话题名称；
        2.QOS(消息队列长度)。
      返回值：发布对象指针。
     */
    // 创建一个发布者 publisher_，用于在话题 chatter 上发布 std_msgs::msg::String 类型的消息，队列大小为10。
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10);
    // 3-2.创建定时器；
    /*  
      参数：
        1.时间间隔；
        2.回调函数；
      返回值：定时器对象指针。
    */
   //  创建一个定时器 timer_，每1秒钟调用一次 on_timer 函数。
    timer_ = this->create_wall_timer(1s,std::bind(&Talker::on_timer,this));

    
  }
private:
  void on_timer(){
    // 3-3.组织并发布消息。
    auto message = std_msgs::msg::String();
    message.data = "hello world!" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(),"发布方发布的消息:%s",message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;
};

int main(int argc, char ** argv)
{
  std::thread([]{
    while (true)
    {
      rclcpp::Rate r(0.3);
      r.sleep();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"-------------");
    }
    
  }).detach();
  // 2.初始化ROS2客户端；
  rclcpp::init(argc,argv);
  // 4.调用spin函数，传入自定义类对象指针；
  rclcpp::spin(std::make_shared<Talker>());
  // 5.释放资源；
  rclcpp::shutdown();

  return 0;
}

