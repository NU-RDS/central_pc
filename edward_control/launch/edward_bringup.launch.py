import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    package_name = "edward_control"

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_vr",
            default_value="false", # TODO: Probably default to True
            description="Choose to use the VR controller"
        ),

        DeclareLaunchArgument(
            "use_can", # TODO: maybe change to 'robot'='real' or 'sim'
            default_value="false", # TODO: default to true after testing
            description="Choose to enable CAN to USB"
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Choose to open RVIZ"
        ),

        # Bring up CAN to USB
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare(package_name),'launch/can_bringup.launch.py']
                )
            ]),
            condition = LaunchConfigurationEquals("use_can", "true"),
        ),

        # state edward_control node
        Node (
            package=package_name,
            executable="edward_control",
            parameters=[{
                #  'use_jsp_gui' : LaunchConfiguration('use_jsp_gui'),
                'use_can' : LaunchConfiguration('use_can')
            }]
        ),

        # start VR publisher node
        Node (
            package="simple_vr_driver",
            executable="VR_publisher",
            condition=LaunchConfigurationEquals("use_vr", "true")
        ),

        # run robot state publisher with robot description defined by the urdf
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description" :
                Command([TextSubstitution(text="xacro "),
                        PathJoinSubstitution(
                        [FindPackageShare(package_name), "urdf/arm.urdf.xacro"])])}
                    ]
        ),

        # start rviz
        Node (
            condition = LaunchConfigurationEquals("use_rviz", "true"),
            package="rviz2",
            executable="rviz2",
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare(package_name),
                    "config/arm.rviz"
                ])
            ]
        )

    ])
