cmake_minimum_required(VERSION 3.8)
project(nav2_benchmark)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_corridor_cylinders.py
  scripts/dynamic_obstacles.py
  scripts/simple_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
