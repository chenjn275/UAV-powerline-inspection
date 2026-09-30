cmake_minimum_required(VERSION 3.8)
project(inspection_gazebo)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/inspection_gazebo_demo.py
  scripts/generate_inspection_world.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
