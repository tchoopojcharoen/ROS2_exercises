#!/usr/bin/python3
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    gain = LaunchConfiguration('gain')
    gain_launch_arg = DeclareLaunchArgument('gain',default_value='10.0')
    turtlesim_control_pkg = get_package_share_directory('turtlesim_control')
    param = os.path.join(turtlesim_control_pkg,'config','leader_config.yaml')
    
    
    leader = Node(
        package='turtlesim_control',
        executable='via_point_follower.py',
        namespace='turtle1',
        parameters=[param]
    )
    
    scheduler = Node(
        package='turtlesim_control',
        executable='scheduler.py',
        namespace='turtle1'
    )
    
    turtle_following_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlesim_control'),
                'launch',
                'turtle_following.launch.py'
            ])
        ]),
        launch_arguments={
            'gain': gain,
            'speed':'0.75'
        }.items()
    )

    launch_description = LaunchDescription()
    launch_description.add_action(gain_launch_arg)
    launch_description.add_action(turtle_following_launch)
    launch_description.add_action(leader)
    launch_description.add_action(scheduler)
    
    
    return launch_description