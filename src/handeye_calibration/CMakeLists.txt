cmake_minimum_required(VERSION 3.8)
project(handeye_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
# 强制使用系统共享版本的 yaml-cpp（/usr/local 的是 non-PIC 静态库，无法链进 .so）
set(yaml-cpp_DIR "/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp" CACHE PATH "" FORCE)
find_package(yaml-cpp REQUIRED NO_DEFAULT_PATH PATHS "/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp")
find_package(tools REQUIRED)
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs)
# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${fmt_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
  ${tools_INCLUDE_DIRS}
)

# 工具库（如果tools是独立库的话）
# add_library(tools SHARED
#   tools/img_tools.cpp
#   tools/math_tools.cpp
#   tools/logger.cpp
#   tools/exiter.cpp
# )
# target_link_libraries(tools ${OpenCV_LIBS} ${Eigen3_LIBS} fmt::fmt)

# 可执行文件：手眼标定节点
add_executable(calibrate_node
  src/calibrate_robotworld_handeye.cpp
)
target_link_libraries(calibrate_node
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  fmt::fmt
  yaml-cpp
  rclcpp::rclcpp
  tools::tools
)
ament_target_dependencies(calibrate_node rclcpp)

# 可执行文件：数据采集节点
add_executable(capture_node
  src/capture.cpp
)
target_link_libraries(capture_node
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  fmt::fmt
  yaml-cpp
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  tools::tools
)
ament_target_dependencies(capture_node rclcpp sensor_msgs cv_bridge)

# 安装
install(TARGETS
  calibrate_node
  capture_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()