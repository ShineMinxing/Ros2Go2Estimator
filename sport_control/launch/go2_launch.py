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
                'ros2 run joy joy_node & '
                'ros2 run sport_control sport_control_node; '
                'read -p "Press enter to close"'
            ],
            output="screen",
        ),

        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "x-terminal-emulator",
                        "--new-process",
                        "-e",
                        'bash',
                        '-c',
                        'source ~/.bashrc && '
                        'ros2 run fusion_estimator fusion_estimator_node;'
                        'read -p "Press enter to close"'
                    ],
                    output="screen",
                ),
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "x-terminal-emulator",
                        "--new-process",
                        "-e",
                        'bash',
                        '-c',
                        'source ~/.bashrc && '
                        'ros2 run dds_rostopic dds_rostopic_node & '
                        'ros2 run message_handle message_handle_node; '
                        'read -p "Press enter to close"'
                    ],
                    output="screen",
                ),
            ]
        ),
    ]
)