import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals

# Sets up CAN to USB stuff by calling some shell commands
def generate_launch_description():

    package_name = 'edward_control'
    script_name = 'can_bringup.sh'

    script_path = PathJoinSubstitution(
        [FindPackageShare(package=package_name), "launch", script_name]
    )

    return LaunchDescription([

        # TODO: add commands to sudoers and get rid of sudo here
        ExecuteProcess(
            cmd=['sudo', 'bash', script_path],
            output='screen'
        ),

        # Start hardware interface node
        Node(
            package=package_name,
            executable="hardware_interface",
        ),

    ])
