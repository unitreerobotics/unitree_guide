import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, LoadComposableNodes
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from launch.substitutions import Command


def launch_setup(context, *args, **kwargs):

    x_spawn = LaunchConfiguration('x_spawn').perform(context)
    y_spawn = LaunchConfiguration('y_spawn').perform(context)
    yaw_spawn = LaunchConfiguration('yaw_spawn').perform(context)
    rname = LaunchConfiguration('rname').perform(context)
    use_sim_time_str =  LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() in ['true', '1', 'yes', 't', 'y']

    print("###############################################################")
    print("SPAWN MULTI Robot="+str(rname) +
          ",["+str(x_spawn)+","+str(y_spawn)+"]")
    print("###############################################################")

    # ROBOT STATE PUBLISHER
    ####### DATA INPUT ##########

    xacro_file = 'robot.xacro'
    package_description = rname + "_description"
    robot_desc_path = os.path.join(get_package_share_directory(
        package_description), "xacro", xacro_file)
    # robot_controllers = os.path.join(get_package_share_directory(
    #     package_description), "config", "robot_control.yaml")

    robot_controllers = os.path.join(get_package_share_directory(
        package_description), "config", "robot_control2.yaml")
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace= "",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace= "",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

        # # Load joint controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name = "controller_manager",    
        namespace= "",
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_desc_path}, robot_controllers],
        output='screen'
    )

    controllers = [
        'FL_hip_controller',
        'FL_thigh_controller',
        'FL_calf_controller',
        'FR_hip_controller',
        'FR_thigh_controller',
        'FR_calf_controller',
        'RL_hip_controller',
        'RL_thigh_controller',
        'RL_calf_controller',
        'RR_hip_controller',
        'RR_thigh_controller',
        'RR_calf_controller'
    ]

    control_spawners = [Node(
        package='controller_manager',
        executable='spawner',
        name=f"controller_spawner_{controller}",
        namespace="",
        arguments=[controller],
        output='screen'
    ) for controller in controllers]

    # Spawn ROBOT Set Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        namespace= "",
        arguments=['-entity',
                   rname,
                   '-x', x_spawn, '-y', y_spawn, '-z', "0.6", '-Y', yaw_spawn,
                   '-topic', 'robot_description',
                   '-timeout', '120.0'
                   ]
    )

    return [robot_state_publisher_node, *control_spawners,controller_manager,joint_state_publisher_node, start_gazebo_ros_spawner_cmd]
    # return [robot_state_publisher_node, *control_spawners, joint_state_publisher_node, start_gazebo_ros_spawner_cmd]




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
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    gazebo_mode_paths = [go1_pkg_share]
    for path in gazebo_mode_paths:
        os.environ["GAZEBO_MODEL_PATH"] += os.pathsep + path
    print(os.environ["GAZEBO_MODEL_PATH"])


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

    # Set the robot_description parameter





    # # Include the unitree_controller launch file
    # set_ctrl_launch = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource([LaunchConfiguration('unitree_controller'), '/launch/set_ctrl.launch']),
    #     launch_arguments={'rname': rname}.items()
    # )

    return LaunchDescription([
        wname,
        use_sim_time,
        gazebo_server,
        gazebo_client,
        x_spawn_arg,
        y_spawn_arg,
        yaw_spawn_arg,
        rname,
        OpaqueFunction(function=launch_setup),
        # urdf_spawner,
        # controller_spawner,
        # robot_state_publisher,
        # set_ctrl_launch
    ])