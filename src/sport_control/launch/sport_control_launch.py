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
                "bash", "-c", "source ~/.bashrc && ros2 run sport_control sport_control_node --ros-args -p network_interface:=enx00e04c8d0eff"
            ],
            output="screen",
        ),

        # 启动 voice_chat_node
         ExecuteProcess(
            cmd=[
                'gnome-terminal', 
                '--window', 
                '--', 
                'bash', 
                '-c', 
                'source /opt/ros/humble/setup.bash && '
                'echo "Voice Window" && '
                'ros2 run voice_chat_py voice_chat_node && '
                'read -p "Press enter to close"'
            ],
            shell=True,
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='message_handle',
            executable='message_handle_node',
            name='message_handle_node',
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=["/home/smx/unitree_ros2_250221/Ros2Go2Estimator/other/slam_params.yaml"]
        ),


        # rviz2 -d /home/smx/unitree_ros2_250221/Ros2Go2Estimator/other/SMXFE_odm.rviz
        # ros2 run rqt_tf_tree rqt_tf_tree
    ])
