#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directory
    pkg_share = FindPackageShare('shortest_path_viz')
    
    # RViz config file path
    rviz_config_file = PathJoinSubstitution([
        pkg_share, 'rviz', 'shortest_path_viz.rviz'
    ])
    
    # Data file path
    data_dir = PathJoinSubstitution([pkg_share])
    
    return LaunchDescription([
        # Launch the visualization node
        Node(
            package='shortest_path_viz',
            executable='shortest_path_visualizer',
            name='shortest_path_visualizer',
            output='screen',
            parameters=[{
                'data_dir': data_dir
            }]
        ),
        
        # Launch RViz2 with custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        
        # Launch static transform publisher for coordinate frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_frame_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        )
    ]) 