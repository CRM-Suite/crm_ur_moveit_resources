cmake_minimum_required(VERSION 3.8)
project(crm_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


### EndEffectorVelocity Message
set(msg_files
  "msg/EndEffectorVelocity.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
 )

ament_export_dependencies(rosidl_default_runtime)

### EndEffectorVelocityPublisher
add_executable(end_effector_velocity_publisher src/end_effector_velocity.cpp)
target_include_directories(end_effector_velocity_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  end_effector_velocity_publisher
  "rclcpp"
  "std_msgs"
  "moveit_ros_planning_interface"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(end_effector_velocity_publisher "${cpp_typesupport_target}")

### Example 1
add_executable(example1 examples/example1.cpp)
target_include_directories(example1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    example1
    "rclcpp"
    "std_msgs"
    "moveit_ros_planning_interface"
)

### Example 2
add_executable(example2 examples/example2.cpp)
target_include_directories(example2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    example2
    "rclcpp"
    "std_msgs"
    "moveit_ros_planning_interface"
)

### Example 3
add_executable(example3 examples/example3.cpp)
target_include_directories(example3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    example3
    "rclcpp"
    "std_msgs"
    "moveit_ros_planning_interface"
)

install(
  DIRECTORY config launch examples 
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(
  TARGETS end_effector_velocity_publisher example1 example2 example3
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS examples/example3.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES .setup_assistant DESTINATION share/${PROJECT_NAME}
)

ament_package()
