cmake_minimum_required(VERSION 3.8)
project(energy_pose_transform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeTargetLocation.srv"
  DEPENDENCIES
    geometry_msgs
)

# 添加可执行文件
add_executable(Pose_Transform_Node src/Pose_Transform_Node.cpp)
ament_target_dependencies(Pose_Transform_Node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  ament_index_cpp
  sensor_msgs
  message_filters
)

# 添加逆变换可执行文件
add_executable(Inverse_Pose_Transform_Node src/Inverse_Pose_Transform_Node.cpp)
ament_target_dependencies(Inverse_Pose_Transform_Node
  rclcpp
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  ament_index_cpp
  sensor_msgs
)
rosidl_target_interfaces(Inverse_Pose_Transform_Node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 虚拟关节状态发布节点（使用 OpenCV 滑动条控制 pitch / yaw）
add_executable(Virtual_Joint_State_Node src/Virtual_Joint_State_Node.cpp)
ament_target_dependencies(Virtual_Joint_State_Node
  rclcpp
  sensor_msgs
)
target_link_libraries(Virtual_Joint_State_Node
  ${OpenCV_LIBS}
)

# 目标位置可视化节点
add_executable(Target_Location_Visualizer_Node src/Target_Location_Visualizer_Node.cpp)
ament_target_dependencies(Target_Location_Visualizer_Node
  rclcpp
  geometry_msgs
  visualization_msgs
)

# 目标位置服务调用节点
add_executable(Target_Location_Caller_Node src/Target_Location_Caller_Node.cpp)
ament_target_dependencies(Target_Location_Caller_Node
  rclcpp
  geometry_msgs
)
rosidl_target_interfaces(Target_Location_Caller_Node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 安装目标
install(TARGETS
  Pose_Transform_Node
  Inverse_Pose_Transform_Node
  Virtual_Joint_State_Node
  Target_Location_Visualizer_Node
  Target_Location_Caller_Node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()