#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    new_background_r = LaunchConfiguration('new_background_r')
    new_background_r_launch_arg = DeclareLaunchArgument(
        'new_background_r',
        default_value='100'
    )

    config = os.path.join(get_package_share_directory('turtlesim_control'),'config','follower_config.yaml')
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[
            {'background_b':100},
            {'background_g':20},
            {'background_r':new_background_r},
        ]
    )

    spawn_turtle2 = ExecuteProcess(
        cmd = [['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],
        shell=True
    )

    follower = Node(
        package='turtlesim_control',
        executable='turtle_follower.py',
        parameters=[
            config
        ],
        remappings=[
            ('/pose','/turtle2/pose'),
            ('/cmd_vel','/turtle2/cmd_vel'),
            ('/goal','/turtle1/pose'),
        ]
    )

    entity_to_run = [turtlesim,spawn_turtle2,follower,new_background_r_launch_arg]
    return LaunchDescription(entity_to_run)

    