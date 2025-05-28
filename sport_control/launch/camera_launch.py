import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction  # 新增 TimerAction

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "x-terminal-emulator",
                "--new-process",
                "-e",
                'bash',
                '-c',
                'source ~/.bashrc && '
                'ros2 run image_transport republish compressed raw   --ros-args     --remap in/compressed:=/SMX/GimbalCamera_Compressed     --remap out:=/SMX/GimbalCamera & '
                'read -p "Press enter to close"'
            ],
            output="screen",
        ),
    ]
)