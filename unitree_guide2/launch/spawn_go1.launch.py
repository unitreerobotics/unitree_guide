import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,RegisterEventHandler,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,PythonExpression, Command
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def launch_setup(context, *args, **kwargs):
        # Get the launch configuration
    package_name='sim_worlds2' #<--- CHANGE ME
    spawn_x_val = LaunchConfiguration('spawn_x_val').perform(context)
    spawn_y_val = LaunchConfiguration('spawn_y_val').perform(context)
    spawn_yaw_val = LaunchConfiguration('spawn_yaw_val').perform(context)
    model_name = 'scout_mini.xacro'
    
    # model_path = os.path.join(get_package_share_directory('scout_description'), "urdf", model_name)
    # xacro_file = os.path.join(get_package_share_directory(package_name),'urdf', 'scout_mini', model_name)
    xacro_file = os.path.join(get_package_share_directory(package_name),'urdf', 'scout_mini', model_name)
    # xacro_file = os.path.join(get_package_share_directory("limo_description"), "urdf/limo_four_diff_3dLidar.xacro")
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', 'true', ' sim_mode:=', "true"])
    
    # # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
        # parameters=[{'robot_description': Command(['xacro ', xacro_file]),'use_sim_time': True}]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', "scout_mini",
                                    '-x', spawn_x_val,
                                    '-y', spawn_y_val,
                                    '-z', "0.3",
                                    '-Y', spawn_yaw_val],
                                     output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )
    delayed_diff_drive_spawner =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    safety_relay_node = Node(
        package="robot_navigation",
        executable="safety_relay",
        name="safety_realy_node",
        respawn=True,
        output="screen"
    ) 
    return [
        node_robot_state_publisher,
        safety_relay_node,
        # twist_mux,
        spawn_entity,
        delayed_joint_broad_spawner,
        delayed_diff_drive_spawner
    ]


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    

   
    ############ You do not need to change anything below this line #############

    # Set the path to different files and folders.  

    # Declare the launch arguments
    declare_spawn_x_val = DeclareLaunchArgument('spawn_x_val', default_value='0.0', description='Spawn X Value')
    declare_spawn_y_val = DeclareLaunchArgument('spawn_y_val', default_value='0.0', description='Spawn Y Value')
    declare_spawn_yaw_val = DeclareLaunchArgument('spawn_yaw_val', default_value='0.00', description='Spawn Yaw Value')

    # Launch them all!
    return LaunchDescription([
        declare_spawn_x_val,
        declare_spawn_y_val,
        declare_spawn_yaw_val,
        OpaqueFunction(function=launch_setup)
    ])
