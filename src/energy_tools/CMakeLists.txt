cmake_minimum_required(VERSION 3.8)
project(energy_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(grab_image_node src/grab_image.cpp)
target_include_directories(grab_image_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(grab_image_node rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(grab_image_node ${OpenCV_LIBS})

add_executable(visualizer_node src/visualizer.cpp)
target_include_directories(visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(visualizer_node rclcpp sensor_msgs geometry_msgs OpenCV message_filters)
target_link_libraries(visualizer_node ${OpenCV_LIBS})

add_executable(roi_visualizer_node src/ROI_visualizer.cpp)
target_include_directories(roi_visualizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(roi_visualizer_node rclcpp sensor_msgs geometry_msgs OpenCV message_filters)
target_link_libraries(roi_visualizer_node ${OpenCV_LIBS})

add_executable(image_viewer_node src/image_viewer.cpp)
target_include_directories(image_viewer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(image_viewer_node rclcpp sensor_msgs OpenCV)
target_link_libraries(image_viewer_node ${OpenCV_LIBS})

install(TARGETS visualizer_node roi_visualizer_node grab_image_node image_viewer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
