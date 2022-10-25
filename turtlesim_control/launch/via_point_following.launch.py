#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    gain = LaunchConfiguration('gain')
    gain_launch_arg = DeclareLaunchArgument('gain',default_value='5.0')
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    controller = Node(
        package='turtlesim_control',
        executable='controller.py',
        namespace='turtle1',
        parameters=[
            {'gain':gain},
            {'speed':2.0},
        ]
    )
    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
        namespace='turtle1'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(gain_launch_arg)
    launch_description.add_action(turtlesim)
    launch_description.add_action(controller)
    launch_description.add_action(scheduler)
    return launch_description