import os

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rname = "go1"
    # Declare launch arguments
    user_debug_arg = DeclareLaunchArgument(
        'user_debug', default_value='false', description='User Debug Argument'
    )


    # Construct robot_description command
    robot_desc_path = os.path.join(get_package_share_directory('go1_description'), 'xacro', 'robot.xacro')
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     namespace=rname,
    #     parameters=[{'robot_description': robot_desc_path}],
    #     output="screen"
    # )
    # # Define Nodes
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[
    #         {'use_gui': True},
    #         {'robot_description': robot_desc_path}
    #     ]
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'publish_frequency': 1000.0},
                    {'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )


    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        # arguments=['-d', os.path.join(get_package_share_directory('go1_description'), 'launch', 'check_joint.rviz')]
    )

    return LaunchDescription([
        user_debug_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
