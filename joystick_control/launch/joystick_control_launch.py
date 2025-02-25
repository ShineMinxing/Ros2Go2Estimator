import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # 修改这里，使用 launch_ros.actions

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
        Node(
            package='joystick_control',
            executable='joystick_control_node',
            name='joystick_control_node',
            output='screen',
            arguments=['enxc8a3627ff10b']  # 替换为您的实际网络接口名称
        ),
    ])
