cmake_minimum_required(VERSION 3.8)
project(px4_offboard_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/px4_offboard_bridge.py
  scripts/super_planner_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
