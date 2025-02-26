import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 自动启动 joy_node
        Node(
            package='joy',           # 包名
            executable='joy_node',   # 可执行文件名
            name='joy_node',         # 节点名
            output='screen',         # 输出到屏幕
        ),
        
        # 启动 joystick_control_node
        ExecuteProcess(
            cmd=[
                "tilix",
                "-e",
                "ros2", "run", "joystick_control", "joystick_control_node",
                "--ros-args",
                "-p", "network_interface:=enxc8a3627ff10b"
            ],
            output="screen",
        ),

        # 启动 fusion_estimator_node
        Node(
            package='fusion_estimator',
            executable='fusion_estimator_node',
            name='fusion_estimator_node',
            output='screen',
            parameters=[{'network_interface': "enxc8a3627ff10b"}],
        ),
    ])
