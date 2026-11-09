#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    aruco_patroller_task_pkg = get_package_share_directory('aruco_patroller_task')

    spawn_markers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco_patroller_task_pkg, 'launch', 'spawn_aruco_markers.launch.py')
        )
    )

    detector_node = Node(
        package='aruco_patroller_task',
        executable='aruco_pose_detector',
        name='aruco_pose_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    patroller_node = Node(
        package='aruco_patroller_task',
        executable='aruco_patroller',
        name='aruco_patroller',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    nodes_sequence = TimerAction(
        period=2.0,
        actions=[detector_node, patroller_node]
    )

    return LaunchDescription([
        spawn_markers_launch,
        nodes_sequence
    ])