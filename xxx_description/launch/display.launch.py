#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    
    full_example_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('turtlesim_control'),
               'launch',
               'full_action.launch.py',
            ]),
        ]),
    )
    

    ld = LaunchDescription()
    ld.add_action(full_example_launch)
    
    return ld
