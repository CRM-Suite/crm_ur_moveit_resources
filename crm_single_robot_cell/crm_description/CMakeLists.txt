cmake_minimum_required(VERSION 3.5)
project(crm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
