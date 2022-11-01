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
import os, yaml
import xacro
from dummy_description.DH2Transform import DH2Transform
class ShareFile():
    def __init__(self,package,folder,file):
        self.package_name = package
        self.package_path = get_package_share_directory(package)
        self.folder = folder
        self.file = file
        self.path = os.path.join(self.package_path,folder,file)
def launch_action_rviz(rviz_config,robot_name=''):
    # generate launch description for rviz
    rviz_file_path = rviz_config.path
    if robot_name:
        new_rviz_file_path = os.path.join(rviz_config.package_path,rviz_config.folder,robot_name+'_'+rviz_config.file)
        with open(rviz_file_path,'r') as file:
            rviz_param = yaml.load(file,yaml.SafeLoader)
        rviz_param['Visualization Manager']['Global Options']['Fixed Frame'] = robot_name+'/base_footprint'
        for display in rviz_param['Visualization Manager']['Displays']:
            if display['Class']=='rviz_default_plugins/RobotModel':
                display['Description Topic']['Value']=robot_name+'/robot_description'
                display['TF Prefix'] = robot_name
        with open(new_rviz_file_path,'w') as file:
            yaml.dump(rviz_param,file)
    else:
        new_rviz_file_path = rviz_file_path
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', new_rviz_file_path],
        output='screen')
    return rviz
def launch_action_robot(robot_description,robot_name=''):
    
    # generate properties yaml file 
    robot_desc_xml = xacro.process_file(robot_description.path).toxml()
    parameters = [{'robot_description':robot_desc_xml}]
    if robot_name:
        parameters.append({'frame_prefix':robot_name+'/'})
    robot_state_publisher = Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='screen',
                                  namespace=robot_name,
                                  parameters=parameters
    )
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=robot_name
    )
    return [robot_state_publisher,joint_state_publisher_gui]
    
def generate_launch_description():
    robot_description = ShareFile('xxx_description','robot','visual/xxx.xacro')
    rviz_config = ShareFile('xxx_description','config','_dummy.rviz')
    
    robot_name = 'dummy_1'
    
    rviz = launch_action_rviz(rviz_config,robot_name)
    actions = launch_action_robot(robot_description,robot_name)
    
    # Launch Description
    launch_description = LaunchDescription()
    for action in actions:
        launch_description.add_action(action)
    launch_description.add_action(rviz)
    
    return launch_description



