#!/usr/bin/env python3

"""
This program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.

created by Thanacha Choopojcharoen at CoXsys Robotics (2022)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from dummy_description.DH2Transform import DH2Transform
class ShareFile():
    def __init__(self,package,folder,file):
        self.package_name = package
        self.package_path = get_package_share_directory(package)
        self.folder = folder
        self.file = file
        self.path = os.path.join(self.package_path,folder,file)
def launch_action_rviz(rviz_config):
    # generate launch description for rviz
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config.path],
        output='screen')
    return rviz
def launch_action_robot(dh_parameters,robot_description):
    
    # generate properties yaml file 
    DH2Transform(dh_parameters.package_name,dh_parameters.folder,dh_parameters.file) 
    robot_desc_xml = xacro.process_file(robot_description.path).toxml()
    parameters = [{'robot_description':robot_desc_xml}]
    
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  parameters=parameters
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    return [robot_state_publisher,joint_state_publisher_gui]
    
def generate_launch_description():
    dh_parameters = ShareFile('dummy_description','config','DH_parameters.yaml')
    robot_description = ShareFile('dummy_description','robot','visual/robot_graphics.xacro')
    rviz_config = ShareFile('dummy_description','config','_dummy.rviz')
    
    
    rviz = launch_action_rviz(rviz_config)
    actions = launch_action_robot(dh_parameters,robot_description)
    
    # Launch Description
    launch_description = LaunchDescription()
    for action in actions:
        launch_description.add_action(action)
    launch_description.add_action(rviz)
    
    return launch_description



