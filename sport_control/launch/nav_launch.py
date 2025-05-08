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
                'ros2 launch nav2_bringup bringup_launch.py '
                'map:=~/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/local_file/map_new.yaml '
                'params_file:=~/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/other/Guide.yaml; '
                'read -p "Press enter to close"'
            ],
            output="screen",
        ),
    ]
)