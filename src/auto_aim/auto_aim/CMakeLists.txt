cmake_minimum_required(VERSION 3.16)
project(auto_aim)

set(OpenVINO_DIR "/opt/intel/openvino_2024/runtime/cmake")


find_package(ament_cmake_auto REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(src/planner/tinympc)

find_package(fmt 8 REQUIRED)
find_package(CURL REQUIRED)
find_package(TIFF REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
include_directories(
  ${YAML_CPP_INCLUDE_DIR}
)


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

ament_auto_find_build_dependencies()

add_library(auto_aim SHARED 
    src/armor.cpp
    src/classifier.cpp 
    src/detector.cpp
    src/solver.cpp
    src/aimer.cpp
    src/target.cpp
    src/tracker.cpp
    src/voter.cpp
    # shooter.cpp
    src/yolo.cpp
    src/yolos/yolov5.cpp
    src/yolos/yolov8.cpp
    src/yolos/yolo11.cpp
    # src/multithread/commandgener.cpp
    # src/multithread/mt_detector.cpp
    src/planner/planner.cpp
)

target_link_libraries(auto_aim
  tools::tools
  rclcpp::rclcpp
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(auto_aim fmt::fmt CURL::libcurl TIFF::TIFF)

target_link_libraries(auto_aim openvino::runtime tinympcstatic)
target_include_directories(auto_aim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS auto_aim tinympcstatic
  EXPORT export_${PROJECT_NAME}  # 必须导出
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 4. 导出 Target（关键！生成 toolsConfig.cmake）
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)


install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()