#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import sys 
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
#sudo apt install ros-foxy-joint-state-publisher
#sudo apt install ros-foxy-joint-state-publisher-gui

def launch_action_rviz(pkg_path,rviz_file_name):
    rviz_file_path = os.path.join(pkg_path, 'rviz', rviz_file_name)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file_path],
        output='screen')
    return rviz
    
def launch_action_robot_state_publisher(pkg_path,robot_file,name):
    urdf = os.path.join(pkg_path, 'robot',robot_file)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[{'robot_description':robot_desc,'frame_prefix':name+'/'}]
    )
    return robot_state_publisher

def launch_action_broadcaster(name):
    tf2_turtle = Node(
        package='turtlesim_control',
        executable='tf_broadcaster.py',
        arguments = [name]  # sent as argument to tf2_broadcaster.py
    )
    return tf2_turtle

def generate_launch_description():
    # Get the launch directory
    description_pkg = get_package_share_directory('xxx_description')
    rviz = launch_action_rviz(description_pkg,'solution_2_robots_config.rviz')
    robot_state_publisher_1 = launch_action_robot_state_publisher(description_pkg,'turtle1.urdf','turtle1')
    robot_state_publisher_2 = launch_action_robot_state_publisher(description_pkg,'turtle1.urdf','turtle2')
    
    
    full_example_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('turtlesim_control'),
               'launch',
               'full_example.launch.py',
            ]),
        ]),
    )

    
    broadcaster_1 = launch_action_broadcaster('turtle1')
    broadcaster_2 = launch_action_broadcaster('turtle2')


    ld = LaunchDescription()
    ld.add_action(full_example_launch)
    ld.add_action(broadcaster_1)
    ld.add_action(broadcaster_2)
    ld.add_action(robot_state_publisher_1)
    ld.add_action(robot_state_publisher_2)
    ld.add_action(rviz)
    
    return ld