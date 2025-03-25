#!/usr/bin/env python3
"""
Launch file for the ODrive control system.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for ODrive control."""
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'config_file',
            default_value='odrive_params.yaml',
            description='ODrive parameters YAML file'),
        DeclareLaunchArgument(
            'joy_config',
            default_value='xbox.yaml',
            description='Joystick configuration file'),
    ]

    # Get the package share directory
    pkg_share = FindPackageShare('odrive_control')
    
    # Get the config file paths
    odrive_config = PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('config_file')])
    joy_config = PathJoinSubstitution([pkg_share, 'config', LaunchConfiguration('joy_config')])
    
    # Create nodes
    odrive_node = Node(
        package='odrive_control',
        executable='odrive_node',
        name='odrive_node',
        parameters=[odrive_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    
    teleop_node = Node(
        package='odrive_control',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    
    # Create the launch description
    ld = LaunchDescription(declared_arguments)
    
    # Add the nodes to the launch description
    ld.add_action(odrive_node)
    ld.add_action(teleop_node)
    ld.add_action(joy_node)
    
    return ld
