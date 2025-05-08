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
                'ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file ~/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/other/slam_params.yaml & '
                'read -p "Press enter to close"'
            ],
            output="screen",
        ),
    ]
)