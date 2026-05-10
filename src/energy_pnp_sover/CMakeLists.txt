cmake_minimum_required(VERSION 3.8)
project(pnp_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include directories: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV not found. Please install OpenCV development package.")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(pnp_solver src/pnp_solver.cpp)

# Add visualizer executable
add_executable(pnp_visualizer src/pnp_visualizer.cpp)

# Link libraries
target_link_libraries(pnp_solver ${OpenCV_LIBS})
target_link_libraries(pnp_visualizer)

# Include directories
target_include_directories(pnp_solver PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(pnp_visualizer PRIVATE)

ament_target_dependencies(pnp_solver rclcpp geometry_msgs sensor_msgs camera_info_manager)
ament_target_dependencies(pnp_visualizer rclcpp geometry_msgs visualization_msgs tf2 tf2_geometry_msgs)

# Install targets
install(TARGETS pnp_solver pnp_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

# Skip testing for now

ament_package()