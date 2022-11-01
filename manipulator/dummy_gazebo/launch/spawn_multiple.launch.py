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
def launch_action_robot_spawner(dh_parameters,robot_description,controller,position,robot_name):
    # robot_state_publisher
    DH2Transform(dh_parameters.package_name,dh_parameters.folder,dh_parameters.file) 
    
    robot_desc_xml = xacro.process_file(robot_description.path,mappings={'robot_name': robot_name}).toxml()
    params = {'robot_description': robot_desc_xml}
    sim_time = {'use_sim_time': True}
    frame_prefix = {'frame_prefix':robot_name+'/'}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name,
        output='both',
        parameters=[params,sim_time,frame_prefix]
    )    
    # controller
    new_config = generate_controller_config(controller,robot_name)
    # spawner
    spawner = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', robot_name+'/robot_description',
            '-entity',robot_name+'/dummy',
            '-x', str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', '0',
            '-P', '0',
            '-Y', '0',
        ]
    )
    #controller_config_path = generate_controller_config(controller,robot_name)
    
    """
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_desc_xml}, controller_config_path],
        output="both",
        namespace=robot_name
    )
    """
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_position_controller", "--controller-manager", robot_name+"/controller_manager"]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", robot_name+"/controller_manager"]
    )
    joint_trajectory_generator = Node(
        package="dummy_control",
        executable="trajectory_generator.py",
        namespace=robot_name,
        parameters=[new_config]
    )
    
    actions = []
    actions.append(robot_state_publisher)
    actions.append(spawner)
    actions.append(joint_trajectory_generator)
    actions.append(joint_trajectory_controller)
    actions.append(joint_state_broadcaster)
    
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
    with open(controller.path) as f:
        doc = yaml.safe_load(f)
    doc[namespace] = dict()
    doc[namespace]['controller_manager'] = recursive_yaml(doc['controller_manager'])
    doc[namespace]['joint_trajectory_position_controller'] = recursive_yaml(doc['joint_trajectory_position_controller'])
    controller_config_path = os.path.join(controller.package_path,controller.folder,namespace+controller.file)
    
    with open(controller_config_path, 'w') as f:
        yaml.dump(doc, f)
    return controller_config_path
def generate_launch_description():
    dh_parameters = ShareFile('dummy_description','config','DH_parameters.yaml')
    robot_description = ShareFile('dummy_gazebo','robot','dummy.xacro')
    controller = ShareFile('dummy_gazebo','config','_controller_config.yaml')
    robot_name = 'dummy_1'
    gazebo_server,gazebo_client = launch_action_gazebo()
    actions_1 = launch_action_robot_spawner(
        dh_parameters,
        robot_description,
        controller,
        [0.0,0.0,0.0],
        robot_name
    )
    launch_description = LaunchDescription()
    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)
    N = 3 # number of robot
    for action in actions_1:
        launch_description.add_action(action)
    previous_actions = actions_1
    for i in range(N-1):
        actions = launch_action_robot_spawner(
            dh_parameters,
            robot_description,
            controller,
            [0.0,-2*(i+1)+0,0.0],
            'dummy_'+str(i+2)
        )
        spawn_event = RegisterEventHandler(
            OnProcessExit(
                target_action=previous_actions[-1],
                on_exit= TimerAction(period = 2.0,actions = actions)
            )
        )
        launch_description.add_action(spawn_event)
        previous_actions = actions
    return launch_description
    
