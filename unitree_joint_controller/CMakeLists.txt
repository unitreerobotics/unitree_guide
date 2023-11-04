# cmake_minimum_required(VERSION 3.8)
# project(unitree_legged_control2)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(controller_interface REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(realtime_tools REQUIRED)
# find_package(ros2_unitree_legged_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(urdf REQUIRED)
# find_package(control_toolbox REQUIRED)

# include_directories(
#   include/unitree_legged_control2
#   ${controller_interface_INCLUDE_DIRS}
#   ${hardware_interface_INCLUDE_DIRS}
#   ${pluginlib_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
#   ${realtime_tools_INCLUDE_DIRS}
#   ${ros2_unitree_legged_msgs_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${urdf_INCLUDE_DIRS}
#   ${control_toolbox_INCLUDE_DIRS}
# )

# # link_directories(lib)

# add_library(unitree_legged_control SHARED
#   src/joint_controller.cpp
# )

# ament_target_dependencies(unitree_legged_control
#   controller_interface
#   hardware_interface
#   pluginlib
#   rclcpp
#   realtime_tools
#   ros2_unitree_legged_msgs
#   geometry_msgs
#   urdf
#   control_toolbox
# )

# ament_export_dependencies(
#   controller_interface
#   hardware_interface
#   pluginlib
#   rclcpp
#   realtime_tools
#   ros2_unitree_legged_msgs
#   geometry_msgs
#   urdf
#   control_toolbox
# )

# ament_export_include_directories(include)
# ament_export_libraries(unitree_legged_control)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# pluginlib_export_plugin_description_file(controller_interface unitree_controller_plugin.xml)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()


cmake_minimum_required(VERSION 3.16)
project(unitree_joint_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
  ros2_unitree_legged_msgs
  control_toolbox
  urdf
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") # Note that it is "aarch64" not "arm64" for ARM architecture checking
    set(LIBRARY_NAME unitree_joint_control_tool_arm64)
else()
    set(LIBRARY_NAME unitree_joint_control_tool)
endif()

find_library(UNITREE_JOINT_CONTROL_TOOL_LIB NAMES ${LIBRARY_NAME} PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib NO_DEFAULT_PATH)
if(NOT UNITREE_JOINT_CONTROL_TOOL_LIB)
    message(FATAL_ERROR "unitree_joint_control_tool library not found")
endif()


find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(unitree_joint_controller_parameters
  src/unitree_joint_controller_parm.yaml
  include/unitree_joint_controller/validate_ujc_parameters.hpp
)

add_library(unitree_joint_controller SHARED
  src/joint_controller.cpp
)
target_compile_features(unitree_joint_controller PUBLIC cxx_std_17)
target_include_directories(unitree_joint_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/unitree_joint_controller>
)

# Here you use the full path provided by UNITREE_JOINT_CONTROL_TOOL_LIB variable
target_link_libraries(unitree_joint_controller PUBLIC unitree_joint_controller_parameters ${UNITREE_JOINT_CONTROL_TOOL_LIB})
ament_target_dependencies(unitree_joint_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(unitree_joint_controller PRIVATE "UNITREE_JOINT_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface unitree_controller_plugin.xml)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   ament_add_gmock(test_diff_drive_controller
#     test/test_diff_drive_controller.cpp
#     ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_diff_drive_controller.yaml)
#   target_link_libraries(test_diff_drive_controller
#     diff_drive_controller
#   )
#   ament_target_dependencies(test_diff_drive_controller
#     geometry_msgs
#     hardware_interface
#     nav_msgs
#     rclcpp
#     rclcpp_lifecycle
#     realtime_tools
#     tf2
#     tf2_msgs
#   )

#   ament_add_gmock(test_load_diff_drive_controller
#     test/test_load_diff_drive_controller.cpp
#   )
#   ament_target_dependencies(test_load_diff_drive_controller
#     controller_manager
#     ros2_control_test_assets
#   )
# endif()

install(
  DIRECTORY include/
  DESTINATION include/unitree_joint_controller
)
install(TARGETS unitree_joint_controller unitree_joint_controller_parameters
  EXPORT export_unitree_joint_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_unitree_joint_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()