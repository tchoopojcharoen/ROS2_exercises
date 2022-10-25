#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    controller = Node(
        package='turtlesim_control',
        executable='controller.py'
    )
    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(turtlesim)
    launch_description.add_action(controller)
    launch_description.add_action(scheduler)
    return launch_description