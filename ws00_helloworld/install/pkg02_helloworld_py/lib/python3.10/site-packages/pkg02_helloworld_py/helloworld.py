"""
  需求：在终端输出文本 hello world。
  流程：
      1.导包；
      2.初始化ROS2客户端；
      3.创建节点；
      4.输出日志；
      5.释放资源。

"""

import rclpy
from rclpy.node import Node

# 方式1(不推荐)
""" 
def main():
    # 2.初始化ROS2客户端；
    rclpy.init()
    # 3.创建节点；
    node = rclpy.create_node("helloworld_node_py")
    # 4.输出日志；
    node.get_logger().info("hello world!(python)")
    # 5.释放资源。
    rclpy.shutdown() 
"""

# 方式2(推荐)
# 自定义类
class MyNode(Node):
    def __init__(self):
        super().__init__("hello_node_py")
        self.get_logger().info("hello world!(Python 继承方式)")

def main():
    # 初始化
    rclpy.init()
    # 创建对象
    node = MyNode()
    # .....
    # 资源释放
    rclpy.shutdown()
    pass


if __name__ == '__main__':
    main()
