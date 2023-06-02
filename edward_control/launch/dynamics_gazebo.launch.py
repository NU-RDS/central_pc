from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import Command, TextSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import ExecuteProcess


def generate_launch_description():

    pkg_share = FindPackageShare("rds_sim")
    world_path = PathJoinSubstitution([pkg_share,'worlds','rds.world'])
    xacro_urdf_path = PathJoinSubstitution([pkg_share,"urdf","arm.urdf.xacro"])
    urdf = Command([TextSubstitution(text="xacro "), xacro_urdf_path])

    ld = LaunchDescription([

        # start gazebo with the world SDF file
        ExecuteProcess(
            cmd = ["ign", "gazebo", world_path]
        ),

        # load the URDF model into gazebo
        Node(
            package = "ros_gz_sim",
            executable = "create",
            arguments= [
                "-topic",
                "/robot_description",
                "-string",
                urdf
            ]
        ),

        # run ros_gz_bridge to connect ign /cmd_vel, /tf, /odom, and /joint_states to ros
        #  Node(
            #  package="ros_gz_bridge",
            #  executable="parameter_bridge",
            #  arguments = [
                #  "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                #  "/world/ddrive_world/model/diff_drive_bot/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model",
                #  "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                #  "/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V"
            #  ],
            #  remappings = [
                #  ("/world/ddrive_world/model/diff_drive_bot/joint_state","/joint_states")
            #  ]
        #  )

    ])

    return ld
