cmake_minimum_required(VERSION 3.10)
project(tools)

link_directories(/usr/lib/x86_64-linux-gnu)  # 替换为你的 fmt 库路径（含 .so 或 .a 文件）

find_package(ament_cmake_auto REQUIRED)
find_package(fmt 8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})


add_library(tools SHARED 
    src/exiter.cpp
    src/extended_kalman_filter.cpp
    src/ransac_sine_fitter.cpp
    src/img_tools.cpp
    src/math_tools.cpp
    src/plotter.cpp
    src/trajectory.cpp
    src/recorder.cpp
    src/logger.cpp
    src/pid.cpp
    src/crc.cpp
    src/ros_time.cpp
)

target_link_libraries(tools PRIVATE fmt::fmt rclcpp::rclcpp Eigen3::Eigen)

target_include_directories(tools
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# 3. 安装库
install(
  TARGETS tools
  EXPORT export_${PROJECT_NAME}  # 必须导出
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 4. 导出 Target（关键！生成 toolsConfig.cmake）
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)

ament_auto_package(
  INSTALL_TO_SHARE
  # USE_SCOPED_HEADER_INSTALL_DIR
)

