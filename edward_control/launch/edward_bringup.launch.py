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
            default_value="true", # TODO: Probably default to True
            description="Choose to use the VR controller"
        ),

        DeclareLaunchArgument(
            "robot",
            default_value="real",
            description="Choose to enable hardware communication if using the real robot"
        ),

        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Choose to open RVIZ"
        ),

        # Bring up hardware interface and can
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution(
                    [FindPackageShare(package_name),'launch/hardware_bringup.launch.py']
                )
            ]),
            condition = LaunchConfigurationEquals("robot", "real"),
        ),

        # start edward_control node
        Node(
            package=package_name,
            executable="edward_control",
            parameters=[{
                'robot' : LaunchConfiguration('robot'),
                'use_vr' : LaunchConfiguration('use_vr')
            }]
        ),

        # start VR publisher node
        #  Node (
            #  package="simple_vr_driver",
            #  executable="VR_publisher",
            #  condition=LaunchConfigurationEquals("use_vr", "true")
        #  ),

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
