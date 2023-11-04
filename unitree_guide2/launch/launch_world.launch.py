# Author: Addison Sears-Collins
# Date: September 27, 2021
# Description: Load an SDF and world file into Gazebo.
# https://automaticaddison.com
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory



def launch_setup(context, *args, **kwargs):
    world_name = LaunchConfiguration('world_name').perform(context)
    # Launch configuration variables specific to simulation
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    # Constants for paths to different files and folders
    package_name = 'unitree_guide2'
    world_file_path = 'worlds/' + world_name + ".sdf"
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Set the path to different files and folders.  
    pkg_gazebo_ros = os.path.join(get_package_share_directory("gazebo_ros"))
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    world_path = os.path.join(pkg_share, world_file_path)


    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,'world': world_path}.items()
    )
 
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    return [
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
    ]
 
def generate_launch_description():
 

     
 
  ############ You do not need to change anything below this line #############
  
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to execute gzclient')
  
  declare_rviz_file_cmd = DeclareLaunchArgument(
    name='rviz_file',
    default_value='limo_nav.rviz',
    description='rviz file name')

             
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  world_name_arg = DeclareLaunchArgument(
     'world_name', 
     default_value='empty', 
     description='Name of the map')
  

  return LaunchDescription([
      world_name_arg,
      declare_rviz_file_cmd,
      declare_use_simulator_cmd,
      declare_simulator_cmd,
      OpaqueFunction(function=launch_setup)

  ])
