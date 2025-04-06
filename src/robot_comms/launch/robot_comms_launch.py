#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    receive_node = Node(
        package='robot_comms',
        executable='receive_robot_node',
        name='receive_robot_node',
        output='screen'
    )
    
    heartbeat_node = Node(
    	package='robot_comms',
        executable='heartbeat_node',
        name='heartbeat_node',
        output='screen'
    )

    transmit_node = Node(
        package='robot_comms',
        executable='transmit_robot_node',
        name='transmit_robot_node',
        output='screen'
    )

    # Delay the start of transmit_robot_node by 50 milliseconds
    delayed_transmit_node = TimerAction(
        period=0.05,  # 50 milliseconds
        actions=[transmit_node]
    )

    return LaunchDescription([
        receive_node,
        heartbeat_node,
        delayed_transmit_node,
    ])
