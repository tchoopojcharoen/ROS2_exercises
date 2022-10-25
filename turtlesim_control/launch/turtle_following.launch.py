#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent, OpaqueFunction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    spawn_turtle2 = ExecuteProcess(
        cmd = [['ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: \'turtle2\'}"']],
        shell=True
    )

    spawn_event = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim,
            on_start=[
                LogInfo(msg='spawning turtle...'),
                spawn_turtle2
            ]
        )
    )

    sim_close_event = RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim,
            on_exit=[
                LogInfo(msg='Turtlesim is closed.'),
                EmitEvent(event=Shutdown(reason='Turtlesim is closed.'))
            ]
        )
    )

    launch_description = LaunchDescription()
    launch_description.add_action(turtlesim)
    launch_description.add_action(spawn_event)
    launch_description.add_action(sim_close_event)
    return launch_description