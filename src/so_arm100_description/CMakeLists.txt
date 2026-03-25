cmake_minimum_required(VERSION 3.8)
project(so_arm100_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
