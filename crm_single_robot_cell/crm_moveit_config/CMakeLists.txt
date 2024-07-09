cmake_minimum_required(VERSION 3.8)
project(crm_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Example 1
add_executable(example1 examples/example1.cpp)
target_include_directories(example1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    example1
    rclcpp
    moveit_ros_planning_interface
)

# Example 2
add_executable(example2 examples/example2.cpp)
target_include_directories(example2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    example2
    rclcpp
    moveit_ros_planning_interface
)

# Test roll motion
add_executable(test-roll-motion examples/test-roll-motion.cpp)
target_include_directories(test-roll-motion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
    test-roll-motion
    rclcpp
    moveit_ros_planning_interface
    tf2_geometry_msgs
)

install(
  DIRECTORY config launch examples 
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(
  TARGETS example1 example2 test-roll-motion
  DESTINATION lib/${PROJECT_NAME}
)

# Python programs
install(PROGRAMS
  examples/example3.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES .setup_assistant DESTINATION share/${PROJECT_NAME}
)

ament_package()
