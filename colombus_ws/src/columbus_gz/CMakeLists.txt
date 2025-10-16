cmake_minimum_required(VERSION 3.5)
project(columbus_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
