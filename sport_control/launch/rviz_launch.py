import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

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
                'rviz2 -d ~/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/other/SMXFE_odm.rviz;'
                'read -p "Press enter to close"'
            ],
            output='screen',
        )
    ]
)