#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare launch argument for map file path
    map_file_arg = DeclareLaunchArgument(
        'map_file_path',
        default_value='/home/ros2/ros2_ws/src/scv_robot_gazebo/maps/20250115_k-city.json',
        description='Path to the JSON map file'
    )
    
    # Include gmserver map service launch
    gmserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gmserver'), 'launch', 'map_service.launch.py')
        ])
    )
    
    # Global path planner node
    global_path_planner_node = Node(
        package='scv_global_planner',
        executable='path_planner_node',
        name='global_path_planner_node',
        output='screen',
        parameters=[{
            'map_file_path': LaunchConfiguration('map_file_path')
        }]
    )
    
    return LaunchDescription([
        map_file_arg,
        gmserver_launch,
        global_path_planner_node,
    ])