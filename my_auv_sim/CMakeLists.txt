cmake_minimum_required(VERSION 3.5)

# Specify the project name and version
project(my_auv_sim VERSION 0.0.1)

# Default to C++14 if not specified
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set the CMAKE_PREFIX_PATH to ensure all necessary packages are found
set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake" ${CMAKE_PREFIX_PATH})
set(ignition-math6_DIR "/usr/lib/x86_64-linux-gnu/cmake/ignition-math6" ${CMAKE_PREFIX_PATH})


# Specify the dependencies for your package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED EXCLUDE_FROM_ALL)  # Avoid loading unnecessary Gazebo parts
find_package(rclpy REQUIRED)  # Add Python dependency for ROS2
find_package(ignition-math6 REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/ignition-math6)


# If your package has additional dependencies, add them here
# find_package(some_other_package REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install models and worlds (if necessary)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Ament package setup
ament_package()

