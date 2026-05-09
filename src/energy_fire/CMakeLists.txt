cmake_minimum_required(VERSION 3.8)
project(fire)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(fitting REQUIRED)
find_package(pose_transform REQUIRED)

# Create executables
add_executable(little_energy_client src/little_energy_client.cpp)
ament_target_dependencies(little_energy_client
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  fitting
  pose_transform
)

add_executable(big_energy_client src/big_energy_client.cpp)
ament_target_dependencies(big_energy_client
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  fitting
  pose_transform
)
target_link_libraries(big_energy_client ${fitting_TARGETS})

# Install targets
install(TARGETS
  little_energy_client
  big_energy_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
