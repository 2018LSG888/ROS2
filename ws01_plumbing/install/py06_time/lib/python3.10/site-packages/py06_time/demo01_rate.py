"""  
    需求：
    流程：
        1.导包；
        2.初始化ROS2客户端；
        3.自定义节点类；
                        
        4.调用spain函数，并传入节点对象；
        5.资源释放。 


"""
# 1.导包；
import threading
import rclpy
from rclpy.node import Node
# from rclpy.timer import Rate
import time


# 3.自定义节点类；
class MyNode(Node):
    def __init__(self):
        super().__init__("mynode_node_py")
        # while rclpy.ok():
        #     self.get_logger().info("+++++++")
        #     time.sleep(1.0)

        self.rate = self.create_rate(1.0)
        thread = threading.Thread(target=self.do_some)
        thread.start()

    def do_some(self):
        while rclpy.ok():
            self.get_logger().info("-========-")
            self.rate.sleep()

def main():
    # 2.初始化ROS2客户端；
    rclpy.init()
    # 4.调用spain函数，并传入节点对象；
    rclpy.spin(MyNode())
    # 5.资源释放。 
    rclpy.shutdown()

if __name__ == '__main__':
    main()