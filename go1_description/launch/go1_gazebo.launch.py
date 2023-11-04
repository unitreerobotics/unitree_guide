import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, LoadComposableNodes
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit

def launch_setup(context, *args, **kwargs):

    x_spawn = LaunchConfiguration('x_spawn').perform(context)
    y_spawn = LaunchConfiguration('y_spawn').perform(context)
    yaw_spawn = LaunchConfiguration('yaw_spawn').perform(context)
    rname = LaunchConfiguration('rname').perform(context)
    use_sim_time =  LaunchConfiguration('use_sim_time')

    print("###############################################################")
    print("SPAWN MULTI Robot="+str(rname) +
          ",["+str(x_spawn)+","+str(y_spawn)+"]")
    print("###############################################################")

    # ROBOT STATE PUBLISHER
    ####### DATA INPUT ##########

    package_description = rname + "_description"
    xacro_file = 'robot.xacro'
    xacro_file = os.path.join(get_package_share_directory(package_description), 'xacro', xacro_file)
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', 'true', ' sim_mode:=', "true"])

    
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
                                   '-entity', rname,
                                    '-x', "0.0",
                                    '-y', "0.0",
                                    '-z', "0.6",
                                    '-Y', "0.0"],
                                     output='screen')

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    FL_hip_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FL_hip_controller"],
    )


    delayed_joint_broad_spawner =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner],
        )
    )
    delayed_FL_hip_controller_spawner =   RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[FL_hip_controller_spawner],
        )
    )


    return [node_robot_state_publisher, spawn_entity, delayed_joint_broad_spawner,delayed_FL_hip_controller_spawner]




def generate_launch_description():
    # Declare launch arguments
    x_spawn_arg = DeclareLaunchArgument('x_spawn', default_value='1.0')
    y_spawn_arg = DeclareLaunchArgument('y_spawn', default_value='0.0')
    yaw_spawn_arg = DeclareLaunchArgument('yaw_spawn', default_value='0.0')
    wname = DeclareLaunchArgument('wname', default_value='earth')
    rname = DeclareLaunchArgument('rname', default_value='go1')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    gazebo_models_path = 'models'
    package_name = 'sim_worlds2'
    world_file_name = 'map_sh.sdf'    
    world = os.path.join(get_package_share_directory('sim_worlds2'),'worlds', world_file_name)
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    go1_pkg_share = FindPackageShare(package="go1_description").find("go1_description")
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    # go1_models_path = os.path.join(go1_pkg_share, "meshes")
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    gazebo_mode_paths = [go1_pkg_share]
    for path in gazebo_mode_paths:
        os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + path
 


    # Include another launch file
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),        
        launch_arguments={
            'world': world
        }.items()
    )
    gazebo_client = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))


    return LaunchDescription([
        wname,
        use_sim_time,
        gazebo_server,
        gazebo_client,
        x_spawn_arg,
        y_spawn_arg,
        yaw_spawn_arg,
        rname,
        OpaqueFunction(function=launch_setup)
    ])