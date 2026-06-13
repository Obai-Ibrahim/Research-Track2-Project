cmake_minimum_required(VERSION 3.8)
project(nav2_benchmark)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  worlds
  maps
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
