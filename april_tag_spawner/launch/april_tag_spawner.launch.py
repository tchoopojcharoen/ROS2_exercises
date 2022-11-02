#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
    
def generate_launch_description():
    entity = LaunchConfiguration('entity') 
    
    entity_launch_arg = DeclareLaunchArgument('entity')
    
    launch_description = LaunchDescription()
    
    return launch_description