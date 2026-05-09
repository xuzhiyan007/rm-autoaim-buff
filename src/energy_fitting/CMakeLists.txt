cmake_minimum_required(VERSION 3.8)
project(fitting)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pose_transform REQUIRED)
find_package(Ceres REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PredictEnergy.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Add executables
add_executable(fitting_node src/fitting_node.cpp)
ament_target_dependencies(fitting_node
  rclcpp
  geometry_msgs
  std_msgs
  OpenCV
)

add_executable(little_energy src/little_energy.cpp)
ament_target_dependencies(little_energy
  rclcpp
  std_msgs
  pose_transform
)

# Ensure service interfaces are generated before building
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(little_energy "${cpp_typesupport_target}")

add_executable(little_energy_client src/little_energy_client.cpp)
ament_target_dependencies(little_energy_client
  rclcpp
  pose_transform
  OpenCV
)
target_link_libraries(little_energy_client "${cpp_typesupport_target}")

add_executable(big_energy src/big_energy.cpp)
ament_target_dependencies(big_energy
  rclcpp
  std_msgs
  geometry_msgs
  pose_transform
)
target_link_libraries(big_energy "${cpp_typesupport_target}" Ceres::ceres)

# add_executable(big_energy_client src/big_energy_client.cpp)
# ament_target_dependencies(big_energy_client
#   rclcpp
#   pose_transform
#   OpenCV
# )
# target_link_libraries(big_energy_client "${cpp_typesupport_target}")

add_executable(big_energy_predict src/big_energy_predict.cpp)
ament_target_dependencies(big_energy_predict
  rclcpp
)
target_link_libraries(big_energy_predict "${cpp_typesupport_target}")

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  fitting_node
  little_energy
  little_energy_client
  big_energy

  big_energy_predict
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
