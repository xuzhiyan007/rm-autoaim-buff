cmake_minimum_required(VERSION 3.8)
project(energy_fitting)

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
find_package(energy_pose_transform REQUIRED)
find_package(Ceres REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PredictEnergy.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

# Add executables
add_executable(fitting_node src/fitting_node.cpp)
target_link_libraries(fitting_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  ${OpenCV_TARGETS}
)

add_executable(little_energy src/little_energy.cpp)
target_link_libraries(little_energy PUBLIC
  ${energy_pose_transform_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

# Ensure service interfaces are generated before building
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(little_energy PUBLIC "${cpp_typesupport_target}")

# add_executable(little_energy_client src/little_energy_client.cpp)
# ament_target_dependencies(little_energy_client
#   rclcpp
#   energy_pose_transform
#   OpenCV
# )
# target_link_libraries(little_energy_client "${cpp_typesupport_target}")

add_executable(big_energy src/big_energy.cpp)
target_link_libraries(big_energy PUBLIC
  ${energy_pose_transform_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)
target_link_libraries(big_energy PUBLIC "${cpp_typesupport_target}" Ceres::ceres)

# add_executable(big_energy_client src/big_energy_client.cpp)
# ament_target_dependencies(big_energy_client
#   rclcpp
#   energy_pose_transform
#   OpenCV
# )
# target_link_libraries(big_energy_client "${cpp_typesupport_target}")

add_executable(big_energy_predict src/big_energy_predict.cpp)
target_link_libraries(big_energy_predict PUBLIC
  rclcpp::rclcpp
)
target_link_libraries(big_energy_predict PUBLIC "${cpp_typesupport_target}")

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install targets
install(TARGETS
  fitting_node
  little_energy
  big_energy
  big_energy_predict
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
