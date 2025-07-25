import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 先在 Python 里把 '~' 展开成 '/home/你的用户名'
    rviz_config = os.path.expanduser(
        '~/ros2_ws/LeggedRobot/src/Ros2Go2Estimator/other/SMXFE_odm.rviz'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )
    ])
