cmake_minimum_required(VERSION 3.5)

project(ros_serial_arm_controllers)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

find_package(orocos_kdl REQUIRED)
find_package(roscpp REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rttest REQUIRED)
find_package(tlsf_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_cmake REQUIRED)

include_directories(include)

rosidl_generate_interfaces(ros_serial_arm_controllers
  "msg/CartesianTrajectory.msg"
  "msg/TorqueCommands.msg"
  "msg/JointStates.msg"
  DEPENDENCIES builtin_interfaces
  DEPENDENCIES geometry_msgs
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(pid_joint_space SHARED
  controllers_ros/src/pid_joint_space.cpp)

target_compile_definitions(pid_joint_space
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pid_joint_space
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(pid_joint_space "pid_joint_space")

set(node_plugins "${node_plugins}pid_joint_space; $<TARGET_FILE:pid_joint_space>\n")


add_library(pid_task_space SHARED
  controllers_ros/src/pid_task_space.cpp)

target_compile_definitions(pid_task_space
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pid_task_space
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(pid_task_space "pid_task_space")

set(node_plugins "${node_plugins}pid_task_space; $<TARGET_FILE:pid_task_space>\n")


add_library(computed_torque_joint_space SHARED
  controllers_ros/src/computed_torque_joint_space.cpp)

target_compile_definitions(computed_torque_joint_space
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(computed_torque_joint_space
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(computed_torque_joint_space "computed_torque_joint_space")

set(node_plugins "${node_plugins}computed_torque_joint_space; $<TARGET_FILE:computed_torque_joint_space>\n")


add_library(operational_space_control SHARED
  controllers_ros/src/operational_space_control.cpp)

target_compile_definitions(operational_space_control
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(operational_space_control
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(operational_space_control "operational_space_control")

set(node_plugins "${node_plugins}operational_space_control; $<TARGET_FILE:operational_space_control>\n")


add_library(ros_gazebo_interface SHARED
  ros_gazebo_interface/src/ros_gazebo_interface.cpp)

target_compile_definitions(ros_gazebo_interface
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ros_gazebo_interface
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(ros_gazebo_interface "ros_gazebo_interface")

set(node_plugins "${node_plugins}ros_gazebo_interface;$<TARGET_FILE:ros_gazebo_interface>\n")


add_library(random_waypoint_generator SHARED
  ros_trajectory_generators/src/random_waypoint_generator.cpp)

ament_target_dependencies(random_waypoint_generator
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(random_waypoint_generator "random_waypoint_generator")

set(node_plugins "${node_plugins}random_waypoint_generator; $<TARGET_FILE:random_waypoint_generator>\n")


add_library(random_waypoint_array_generator SHARED
  ros_trajectory_generators/src/random_waypoint_array_generator.cpp)

ament_target_dependencies(random_waypoint_array_generator
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(random_waypoint_array_generator "random_waypoint_array_generator")

set(node_plugins "${node_plugins}random_waypoint_array_generator; $<TARGET_FILE:random_waypoint_array_generator>\n")


add_library(cartesian_trajectory_generator SHARED
  ros_trajectory_generators/src/cartesian_trajectory_generator.cpp)

ament_target_dependencies(cartesian_trajectory_generator
  "class_loader"
  "rclcpp"
  "std_msgs")

rclcpp_register_node_plugins(cartesian_trajectory_generator "cartesian_trajectory_generator")

set(node_plugins "${node_plugins}cartesian_trajectory_generator; $<TARGET_FILE:cartesian_trajectory_generator>\n")


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")


add_executable(computed_torque_comp
  composition/computed_torque_comp.cpp)

target_link_libraries(computed_torque_comp
  pid_joint_space
  pid_task_space
  computed_torque_joint_space
  operational_space_control
  ros_gazebo_interface)

ament_target_dependencies(computed_torque_comp
  "rclcpp")


add_executable(operational_space_comp
  composition/operational_space_comp.cpp)

target_link_libraries(operational_space_comp
  pid_joint_space
  pid_task_space
  computed_torque_joint_space
  operational_space_control
  ros_gazebo_interface)

ament_target_dependencies(operational_space_comp
  "rclcpp")


add_executable(pid_joint_space_comp
  composition/pid_joint_space_comp.cpp)

target_link_libraries(pid_joint_space_comp
  pid_joint_space
  pid_task_space
  computed_torque_joint_space
  operational_space_control
  ros_gazebo_interface)

ament_target_dependencies(pid_joint_space_comp
  "rclcpp")


add_executable(pid_task_space_comp
  composition/pid_task_space_comp.cpp)

target_link_libraries(pid_task_space_comp
  pid_joint_space
  pid_task_space
  computed_torque_joint_space
  operational_space_control
  ros_gazebo_interface)

ament_target_dependencies(pid_task_space_comp
  "rclcpp")


install(TARGETS
  pid_joint_space
  pid_task_space
  computed_torque_joint_space
  operational_space_control
  ros_gazebo_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  pid_task_space_comp
  pid_joint_space_comp
  operational_space_comp
  computed_torque_comp
  DESTINATION lib/${PROJECT_NAME})

ament_package()
