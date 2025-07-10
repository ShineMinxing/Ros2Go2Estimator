from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # gimbal 节点
        Node(
            package='g1_gimbal',
            executable='g1_gimbal_node',
            output='screen',
        ),
        # camera 节点
        Node(
            package='g1_camera',
            executable='g1_camera_node',
            output='screen',
        ),
    ])
