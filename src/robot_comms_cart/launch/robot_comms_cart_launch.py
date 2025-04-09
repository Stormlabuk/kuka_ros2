#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    receive_node = Node(
        package='robot_comms_cart',
        executable='receive_robot_node',
        name='receive_robot_node',
        output='screen'
    )
    
    heartbeat_node = Node(
        package='robot_comms_cart',
        executable='heartbeat_node',
        name='heartbeat_node',
        output='screen'
    )

    transmit_node = Node(
        package='robot_comms_cart',
        executable='transmit_robot',
        name='transmit_robot_node',
        output='screen'
    )

    pause_node = Node(        
        package='robot_comms_cart',
        executable='pause_controller',
        name='pause_node',
        output='screen'
    )

    delayed_transmit_node = TimerAction(
        period=0.05,  # 50 ms delay
        actions=[transmit_node]
    )

    return LaunchDescription([
        receive_node,
        heartbeat_node,
        delayed_transmit_node,
        pause_node
    ])
