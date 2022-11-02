#!usr/bin/python3

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

import os, yaml
import xacro
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit, OnExecutionComplete
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from dummy_description.DH2Transform import DH2Transform
class ShareFile():
    def __init__(self,package,folder,file):
        self.package_name = package
        self.package_path = get_package_share_directory(package)
        self.folder = folder
        self.file = file
        self.path = os.path.join(self.package_path,folder,file)
def launch_action_gazebo():
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ])
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )
    return gazebo_server,gazebo_client
def launch_action_robot_spawner(dh_parameters,robot_description,position):
    # robot_state_publisher
    DH2Transform(dh_parameters.package_name,dh_parameters.folder,dh_parameters.file) 
    parameters = []
    robot_desc_xml = xacro.process_file(robot_description.path).toxml()
    parameters.append({'robot_description':robot_desc_xml})
    parameters.append({'use_sim_time': True})
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=parameters
    )    
    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic','/robot_description',
            '-entity','/dummy',
            '-x',str(position[0]),
            '-y',str(position[1]),
            '-z',str(position[2]),
            '-R','0',
            '-P','0',
            '-Y','0',
        ]
    )
    
    actions = []
    actions.append(robot_state_publisher)
    actions.append(spawner)
    
    return actions
def recursive_yaml(value):
    if isinstance(value,dict):
        new_dict = dict()
        for k,v in value.items():
            new_dict[k] = recursive_yaml(v)
        return new_dict
    if isinstance(value,list):
        new_list = list()
        for e in value:
            new_list.append(e)
        return new_list
    else:
        return value
def generate_controller_config(controller,namespace):
    pass
def generate_launch_description():
    dh_parameters = ShareFile('dummy_description','config','DH_parameters.yaml')
    robot_description = ShareFile('dummy_gazebo','robot','dummy.xacro')
    
    gazebo_server,gazebo_client = launch_action_gazebo()
    actions = launch_action_robot_spawner(
        dh_parameters,
        robot_description,
        [0.0,0.0,0.0]
    )
    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)
    for action in actions:
        launch_description.add_action(action)
    
    return launch_description
    
