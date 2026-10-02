cmake_minimum_required(VERSION 3.8)
project(inspection_manager)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

install(PROGRAMS
  inspection_manager/inspection_manager_node.py
  inspection_manager/perception_sim_node.py
  inspection_manager/tower_pointcloud_node.py
  inspection_manager/tower_pointcloud_sim_node.py
  inspection_manager/inspection_sensor_sim_node.py
  inspection_manager/obstacle_pointcloud_sim_node.py
  inspection_manager/vision_observation_sim_node.py
  inspection_manager/quality_coverage_sim_node.py
  inspection_manager/pointcloud_image_fusion_sim_node.py
  inspection_manager/health_fault_sim_node.py
  inspection_manager/px4_position_odom_bridge.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
# The node imports the geometry helper as a regular Python package when run
# from the installed ROS prefix.
install(DIRECTORY inspection_manager DESTINATION lib/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py")
ament_package()
