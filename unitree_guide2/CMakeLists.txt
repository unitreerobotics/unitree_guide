cmake_minimum_required(VERSION 3.14)
project(unitree_guide2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROBOT_TYPE Go1)
set(PLATFORM amd64)

set(SIMULATION ON)
set(REAL_ROBOT OFF)
set(DEBUG OFF)
set(MOVE_BASE OFF)

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(SIMULATION OR MOVE_BASE)
    add_definitions(-DRUN_ROS)
endif()

set(CMAKE_CXX_STANDARD 11)
find_package(Boost REQUIRED)

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python3 COMPONENTS Interpreter Development NumPy)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(genmsg REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)
if(SIMULATION)
  add_definitions(-DCOMPILE_WITH_SIMULATION)
  find_package(gazebo_ros_pkgs REQUIRED)
  find_package(controller_manager REQUIRED)
#   find_package(joint_state_publisher REQUIRED)
endif()
add_definitions(-DCOMPILE_WITH_ROS)

include_directories(
    include/unitree_guide2
    ${Boost_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(library/unitree_legged_sdk_3.2/include)
        link_directories(library/unitree_legged_sdk_3.2/lib)
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(library/unitree_legged_sdk-3.8.0/include)
        if(${PLATFORM} STREQUAL "amd64")
            link_directories(library/unitree_legged_sdk-3.8.0/lib/cpp/amd64)
        elseif(${PLATFORM} STREQUAL "arm64")
            link_directories(library/unitree_legged_sdk-3.8.0/lib/cpp/arm64)
        endif()
    endif()
endif()

file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})
ament_target_dependencies(junior_ctrl rclcpp  sensor_msgs std_msgs tf2 tf2_ros geometry_msgs ros2_unitree_legged_msgs)

if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python3::Python Python3::NumPy)
endif()

if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    endif()
endif()

target_link_libraries(junior_ctrl -pthread lcm)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)


install(
  TARGETS junior_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


