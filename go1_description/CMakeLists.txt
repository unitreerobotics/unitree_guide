cmake_minimum_required(VERSION 3.5)
project(go1_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
    # include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

# If you have any executables or libraries, you can specify them here.
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs tf2 tf2_ros)
install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    xacro
  DESTINATION
    share/${PROJECT_NAME}/
)
ament_package()
