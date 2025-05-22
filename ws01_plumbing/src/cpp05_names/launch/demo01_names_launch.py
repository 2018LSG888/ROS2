from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 修改节点名称
    # return LaunchDescription([
    #     Node(package="turtlesim",executable= "turtlesim_node", name="turtlesim1"),
    #     Node(package="turtlesim",executable= "turtlesim_node", namespace="t2"),
    #     Node(package="turtlesim",executable= "turtlesim_node", namespace="t3",name="turtlesim3"),
    # ])

    # 修改话题名称
    return LaunchDescription([
        Node(package="turtlesim",executable= "turtlesim_node", name="t1"),
        Node(package="turtlesim",executable="turtlesim_node", 
             remappings=[("/turtle1/cmd_vel","/cmd_vel")],
        )
    ])