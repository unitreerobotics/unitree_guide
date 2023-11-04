import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,RegisterEventHandler,OpaqueFunction,ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,PythonExpression,Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def launch_setup(context, *args, **kwargs):
        # Get the launch configuration
    package_name='go1' #<--- CHANGE ME
    spawn_x_val = LaunchConfiguration('spawn_x_val').perform(context)
    spawn_y_val = LaunchConfiguration('spawn_y_val').perform(context)
    spawn_yaw_val = LaunchConfiguration('spawn_yaw_val').perform(context)
    spawn_yaw_val = LaunchConfiguration('spawn_yaw_val').perform(context)
    # use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = os.path.join(get_package_share_directory("go1_description"))
    default_model_path = os.path.join(pkg_share, "xacro/robot.xacro")



    robot_description_config = Command(['xacro ', default_model_path])
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', package_name,
                                    '-x', spawn_x_val,
                                    '-y', spawn_y_val,
                                    '-z', "0.2",
                                    '-Y', spawn_yaw_val],
                                     output='screen')

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_states_controller"],
        output='screen'
    )


    load_joint_trajectory_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FL_hip_controller"],
        output='screen'
    )

    delayed_joint_broad_spawner =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )
    delayed_diff_drive_spawner =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_trajectory_effort_controller],
        )
    )

    controller_names = [
        "FL_hip_controller", "FL_thigh_controller", "FL_calf_controller",
        "FR_hip_controller", "FR_thigh_controller", "FR_calf_controller",
        "RL_hip_controller", "RL_thigh_controller", "RL_calf_controller",
        "RR_hip_controller", "RR_thigh_controller", "RR_calf_controller"
    ]

    # Create a list to hold the event handlers for the controllers
    controller_event_handlers = []

    # Create the load and event handler actions for each controller
    for controller_name in controller_names:
        load_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name],
            output='screen'
        )
        # controller_spawners.append(load_controller)
        
        # Create an event handler to load each controller after the previous one has finished
        delayed_diff_drive_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_controller],
            )
        )
        controller_event_handlers.append(delayed_diff_drive_spawner)

    # Add all nodes and event handlers to the launch description
    ld = [
        node_robot_state_publisher,
        spawn_entity,
        delayed_joint_broad_spawner,
    ] + controller_event_handlers


    return ld


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    

   
    ############ You do not need to change anything below this line #############

    # Set the path to different files and folders.  

    # Declare the launch arguments
    declare_spawn_x_val = DeclareLaunchArgument('spawn_x_val', default_value='0.0', description='Spawn X Value')
    declare_spawn_y_val = DeclareLaunchArgument('spawn_y_val', default_value='0.0', description='Spawn Y Value')
    declare_spawn_yaw_val = DeclareLaunchArgument('spawn_yaw_val', default_value='0.00', description='Spawn Yaw Value')
    declare_use_sim_time =    DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true')
    # Launch them all!
    return LaunchDescription([
        declare_spawn_x_val,
        declare_spawn_y_val,
        declare_spawn_yaw_val,
        declare_use_sim_time,
        OpaqueFunction(function=launch_setup)
    ])
