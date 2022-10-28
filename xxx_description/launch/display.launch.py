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

def launch_action_robot_state_publisher(pkg_path,robot_file):
    file_path = os.path.join(pkg_path, 'robot',robot_file)
    if robot_file.endswith('.urdf'):
        with open(file_path, 'r') as infp:
            robot_desc = infp.read()
    elif robot_file.endswith('.xacro'):
        robot_desc = xacro.process_file(file_path).toxml()
        #robot_desc = xacro.process_file(robot_desc_path,mappings={'robot_name' : 'xxx1'}).toxml()
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=[{'robot_description':robot_desc,'frame_prefix':'turtle1/'}]
    )
    return robot_state_publisher


def generate_launch_description():
    # Get the launch directory
    description_pkg = get_package_share_directory('xxx_description')
    rviz = launch_action_rviz(description_pkg,'config.rviz')
    robot_state_publisher = launch_action_robot_state_publisher(description_pkg,'visual/turtle1.xacro')
    
    
    full_example_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
               FindPackageShare('turtlesim_control'),
               'launch',
               'full_action.launch.py',
            ]),
        ]),
    )

    tf2_turtle = Node(
        package='turtlesim_control',
        executable='tf_broadcaster.py',
        arguments = ['turtle1']  # sent as argument to tf2_broadcaster.py
    )
    

    ld = LaunchDescription()
    ld.add_action(full_example_launch)
    ld.add_action(tf2_turtle)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz)
    
    return ld