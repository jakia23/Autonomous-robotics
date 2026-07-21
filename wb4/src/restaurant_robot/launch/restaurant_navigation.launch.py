#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg      = get_package_share_directory('restaurant_robot')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    gz_pkg   = get_package_share_directory('gazebo_ros')

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_rviz     = LaunchConfiguration('use_rviz')

    urdf_path   = os.path.join(pkg, 'urdf',   'waiter_robot.urdf.xacro')
    world_file  = os.path.join(pkg, 'worlds', 'restaurant.world')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz',   'restaurant_nav.rviz')
    map_yaml    = os.path.join(pkg, 'maps',   'restaurant_map.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
               output='screen')

    # Robot spawns at y=4.5 — in open restaurant area
    # Clearances: counter(y=3.25)=1.25m, N-wall(y=6.0)=1.5m — both >> 0.52m needed
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-topic', '/robot_description', '-entity', 'waiter_robot',
                             '-x', '0.0', '-y', '3.5', '-z', '0.1',
                             '-R', '0', '-P', '0', '-Y', '3.1416'],
                 output='screen')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml, 'use_sim_time': 'true',
                          'params_file': nav2_params, 'autostart': 'true'}.items())

    rviz = Node(package='rviz2', executable='rviz2',
                arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}],
                condition=IfCondition(use_rviz), output='screen')

    return LaunchDescription([
        use_rviz_arg, gazebo, rsp,
        TimerAction(period=3.0,  actions=[spawn]),
        TimerAction(period=5.0,  actions=[nav2]),
        TimerAction(period=10.0, actions=[rviz]),
    ])
