# Copyright 2023 Ingot Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_ros

def generate_launch_description():

    # Removed the URG node launch (laser_node)

    # Include the robot model description
    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot2_description'),
                'launch',
                'turtlebot2_description.launch.py'
            ])
        ])
    )

    # Define the Kobuki node
    kobuki_node = GroupAction(
        actions=[
            launch_ros.actions.SetRemap(src='/commands/velocity', dst='/cmd_vel'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('kobuki_node'),
                        'launch',
                        'kobuki_node-launch.py'
                    ])
                ])
            )
        ]
    )

    # Robot localization node (commented out in original)
    robot_localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot2_bringup'),
                'launch',
                'ekf_node.launch.py'
            ])
        ])
    )

    # Define the SLAM node with remapping
    slam_node = GroupAction(
        actions=[
            launch_ros.actions.SetRemap(src='/scan', dst='/scan_filtered'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    ])
                ])
            )
        ]
    )

    # Navigation node
    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ])
    )

    # Laser filter node
    laser_filter = LaunchDescription()
    laser_filter_node = launch_ros.actions.Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('turtlebot2_bringup'),
                'config',
                'turtlebot2_laser_scan_filter.yaml'
            ])
        ],
    )
    laser_filter.add_action(laser_filter_node)

    # Realsense camera node
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
        'pointcloud.enable': 'true',
        'align_depth.enable': 'true',  # Example of another parameter
        'enable_sync': 'true',
        'filters': 'pointcloud',
        'initial_reset': 'true'
        'color_width': '1280',
        'color_height': '720',
        'color_fps': '30',
        'depth_width': '1280',
        'depth_height': '720',
        'depth_fps': '30'
    }.items()
    )

    # Added the xv11 LIDAR node
    xv11_lidar_node = launch_ros.actions.Node(
        package='xv11_lidar_python',
        executable='xv11_lidar',
        name='xv11_lidar_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',       # Adjust the port if necessary
            'frame_id': 'laser_frame'     # Adjust the frame_id if necessary
        }],
        remappings=[
            ('/scan', '/scan')            # Ensure the topic names match
        ]
    )

    return LaunchDescription([
        robot_model,
        xv11_lidar_node,      # Included the new xv11 node
        laser_filter,
        realsense_node,
        kobuki_node,
        # robot_localization_node,  # Uncomment if needed
        slam_node,
        nav_node
    ])
