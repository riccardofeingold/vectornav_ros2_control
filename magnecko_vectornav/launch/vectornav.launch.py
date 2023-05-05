import imp
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("magnecko_vectornav"),
                    "description",
                    "vectornav_control.urdf.xacro"
                ]
            )
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("magnecko_vectornav"),
            "config",
            "vectornav_controller.yaml",
        ]
    )


    imu_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["magnecko_imu_broadcaster", "-c", "/controller_manager"],
        output="both",
    )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    
    ld = LaunchDescription([
        control_node,
        imu_broadcaster,
    ])

    return ld
