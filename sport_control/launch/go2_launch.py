import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction  # 新增 TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 第 1 组：立即启动
        Node(
            package='voice_chat',
            executable='voice_chat_node',
            name='voice_chat_node',
            output='screen'
        ),

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

        TimerAction(
            period=3.0,
            actions=[
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
        ),
    ]
)

# ros2 run nav2_map_server map_saver_cli -f /home/unitree/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/local_file/new_map --fmt png
# rviz2 -d ~/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/other/SMXFE_odm.rviz
# ros2 run rqt_tf_tree rqt_tf_tree