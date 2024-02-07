import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    dobot_type = os.getenv('DOBOT_TYPE', 'me6')

    packages_name = "dobot_description"
    robot_description_file_name = dobot_type + "_robot.urdf"
    rviz_file_name = "urdf.rviz"

    print("robot model file: {}".format(robot_description_file_name))
    
    robot_model_path = PathJoinSubstitution(
        [FindPackageShare(packages_name), "urdf", robot_description_file_name]
    )
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_model_path
        ]
    )

    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(packages_name), "rviz", rviz_file_name]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=[robot_model_path]
    )
    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes = [
        rviz_node,
        robot_state_pub_node,
        joint_state_pub_gui_node,
    ]

    return LaunchDescription(nodes)