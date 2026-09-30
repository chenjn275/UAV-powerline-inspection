cmake_minimum_required(VERSION 3.5)

project(mars_quadrotor_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
        set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
        set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Collect message files
set(msg_files
        "${CMAKE_CURRENT_SOURCE_DIR}:msg/QuadrotorState.msg"
        "${CMAKE_CURRENT_SOURCE_DIR}:msg/TrakingPerformance.msg"
        "${CMAKE_CURRENT_SOURCE_DIR}:msg/PositionCommand.msg"
        "${CMAKE_CURRENT_SOURCE_DIR}:msg/PolynomialTrajectory.msg"
        "${CMAKE_CURRENT_SOURCE_DIR}:msg/MpcPositionCommand.msg"
)
# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs builtin_interfaces)

# Finalize package
ament_package()
