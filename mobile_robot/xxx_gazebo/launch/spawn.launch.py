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
def launch_action_robot_spawner(robot_description,controller,position,robot_name=''):
    # robot_state_publisher
    parameters = []
    if robot_name:
        robot_desc_xml = xacro.process_file(robot_description.path,mappings={'robot_name':robot_name}).toxml()
        parameters.append({'frame_prefix':robot_name+'/'})
    else:
        robot_desc_xml = xacro.process_file(robot_description.path).toxml()
    parameters.append({'robot_description':robot_desc_xml})
    parameters.append({'use_sim_time': True})
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=parameters,
        namespace=robot_name
    )    

    generate_controller_config(controller,robot_name)

    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', robot_name+'/robot_description',
            '-entity', robot_name+'/xxx',
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ]
    )

    velocity_controllers = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['velocity_controllers','--controller-manager',robot_name+'/controller_manager']
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster','--controller-manager',robot_name+'/controller_manager']
    )

    
    actions = []
    actions.append(robot_state_publisher)
    actions.append(spawner)
    actions.append(velocity_controllers)
    actions.append(joint_state_broadcaster)

    return actions
def generate_controller_config(controller:ShareFile,namespace):
    with open(controller.path,'r') as file:
        doc = yaml.load(file,yaml.SafeLoader)
    ###
    new_doc = {namespace:doc}
    ###
    new_controller_path = os.path.join(controller.package_path,controller.folder,namespace+controller.file)
    with open(new_controller_path,'w') as file:
        yaml.dump(new_doc,file)
    return new_controller_path
def generate_launch_description():
    
    robot_description = ShareFile('xxx_gazebo','robot','xxx.xacro')
    controller = ShareFile('xxx_gazebo','config','_controller_config.yaml')
    
    gazebo_server,gazebo_client = launch_action_gazebo()
    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)
    
    N = 1
    for i in range(N):
        robot_name = 'xxx_'+str(i+1)
        actions = launch_action_robot_spawner(
            robot_description,
            controller,
            [0.0,-2*i,0.0],
            robot_name
        )
        if i == 0 :
            for action in actions:
                launch_description.add_action(action)  
        else:
            spawn_event = RegisterEventHandler(
                OnProcessExit(
                    target_action=previous_action,
                    on_exit= TimerAction(period=2.0,actions=actions)
                )
            )
            launch_description.add_action(spawn_event)
        previous_action = actions[-1]

    return launch_description
    
