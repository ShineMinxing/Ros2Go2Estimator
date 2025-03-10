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
        
        # 启动 fusion_estimator_node
        ExecuteProcess(
            cmd=[
                "x-terminal-emulator",
                "--new-process",
                "-e",
                "bash", "-c", "source ~/.bashrc && ros2 run fusion_estimator fusion_estimator_node --ros-args -p network_interface:=enx00e04c8d0eff"
            ],
            output="screen",
        ),

        # 启动 joystick_control_node
        ExecuteProcess(
            cmd=[
                "x-terminal-emulator",
                "--new-process",
                "-e",
                "bash", "-c", "source ~/.bashrc && ros2 run joystick_control joystick_control_node --ros-args -p network_interface:=enx00e04c8d0eff"
            ],
            output="screen",
        ),

        # 启动 voice_chat_node
        ExecuteProcess(
            cmd=[
                "x-terminal-emulator",
                "--new-process",
                "-e",
                "bash", "-c", "source ~/.bashrc && ros2 run voice_chat_py voice_chat_node"
            ],
            output="screen",
        ),
    ])
