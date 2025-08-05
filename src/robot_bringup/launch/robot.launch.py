#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package directories
    robot_bringup_dir = FindPackageShare('robot_bringup')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Whether to run SLAM'
    )
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')
    
    # RoboClaw motor driver
    roboclaw_node = Node(
        package='roboclaw_driver',
        executable='roboclaw_node',
        name='roboclaw_driver',
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud_rate': 38400,
            'address': 0x80,
            'wheel_separation': 0.5,  # Adjust for your robot
            'wheel_radius': 0.1,      # Adjust for your robot
            'max_speed': 1.0,
            'ticks_per_meter': 1000,  # Adjust for your encoders
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom')
        ]
    )
    
    # Intel RealSense camera
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_rgbd': 'true',
            'enable_sync': 'true',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Static transform from base_link to camera
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )
    
    # RTAB-Map SLAM
    rtabmap_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ])
        ]),
        condition=IfCondition(slam),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start --Optimizer/GravitySigma 0.3',
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'approx_sync': 'false',
            'wait_imu_to_init': 'false',
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Obstacle avoidance node
    obstacle_avoidance_node = Node(
        package='robot_bringup',
        executable='obstacle_avoidance.py',
        name='obstacle_avoidance',
        parameters=[{
            'min_distance': 0.6,  # 2 feet = ~0.6 meters
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/scan', '/scan'),
            ('/cmd_vel_in', '/cmd_vel_nav'),
            ('/cmd_vel_out', '/cmd_vel')
        ]
    )
    
    # Navigation2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': PathJoinSubstitution([
                robot_bringup_dir,
                'config',
                'nav2_params.yaml'
            ]),
            'map': map_file
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        slam_arg,
        map_arg,
        roboclaw_node,
        realsense_node,
        camera_tf,
        rtabmap_slam,
        obstacle_avoidance_node,
        nav2_launch
    ])