cmake_minimum_required(VERSION 3.16)
project(energy_openvino_infer)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

# 为共享库组件显式查找系统动态版 yaml-cpp，避免静态库未编译 -fPIC 导致链接失败
find_library(YAML_CPP_SHARED_LIB
    NAMES yaml-cpp
    PATHS /usr/lib/x86_64-linux-gnu /usr/lib
    NO_DEFAULT_PATH
)
if(NOT YAML_CPP_SHARED_LIB)
    message(WARNING "Shared yaml-cpp not found, falling back to default (may cause -fPIC errors in shared libs)")
    set(YAML_CPP_SHARED_LIB yaml-cpp)
endif()
message(STATUS "Using yaml-cpp shared lib for component: ${YAML_CPP_SHARED_LIB}")

# Detect ROS2 version and set appropriate macro definitions
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
  message(STATUS "Detected ROS2 distribution: ${ROS_DISTRO}")

  if(ROS_DISTRO STREQUAL "rolling" OR ROS_DISTRO STREQUAL "kilted")
    add_definitions(-DROS2_ROLLING)
    message(STATUS "Setting ROS2_ROLLING flag for ${ROS_DISTRO} distribution")
    set(CMAKE_CXX_STANDARD 17)
  elseif(ROS_DISTRO STREQUAL "humble")
    add_definitions(-DROS2_HUMBLE)
    add_compile_definitions(ROS_DISTRO_HUMBLE)
    message(STATUS "Setting ROS2_HUMBLE flag for Humble distribution")
    set(CMAKE_CXX_STANDARD 14)
  else()
    message(WARNING "Unsupported ROS2 distribution: ${ROS_DISTRO}. Using default settings.")
    set(CMAKE_CXX_STANDARD 14)
  endif()
else()
  message(WARNING "ROS_DISTRO environment variable not found. Using default settings.")
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set compilation optimization options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ffast-math -funroll-loops")

# Architecture-specific compiler optimizations
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mtune=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mavx2")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd")
endif()

# Enable OpenMP support
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find OpenCV package
find_package(OpenCV REQUIRED)
option(WITH_OPENMP "Enable OpenMP support" ON)
option(ENABLE_FAST_MATH "Enable fast math operations" ON)
option(WITH_TBB "Enable TBB support" ON)

# Find OpenVINO (optional)
# Try CMake config first, then pkg-config
set(OPENVINO_AVAILABLE FALSE)

# Auto-detect OpenVINO CMake config path if not found
if(NOT OpenVINO_DIR)
    # Search in common installation paths
    file(GLOB OPENVINO_CMAKE_PATHS
        "/usr/lib/cmake/openvino*/OpenVINOConfig.cmake"
        "/opt/intel/openvino*/OpenVINOConfig.cmake"
        "/opt/openvino*/OpenVINOConfig.cmake"
    )
    if(OPENVINO_CMAKE_PATHS)
        list(GET OPENVINO_CMAKE_PATHS 0 OPENVINO_CMAKE_FILE)
        get_filename_component(OpenVINO_DIR ${OPENVINO_CMAKE_FILE} DIRECTORY)
        message(STATUS "Auto-detected OpenVINO_DIR: ${OpenVINO_DIR}")
    endif()
endif()

find_package(OpenVINO QUIET)

if(OpenVINO_FOUND)
    message(STATUS "Found OpenVINO via CMake config")
    set(OPENVINO_AVAILABLE TRUE)
    set(OPENVINO_LIBS openvino::runtime)
    # Get library directory for RPATH
    get_target_property(OPENVINO_LIB_PATH openvino::runtime IMPORTED_LOCATION)
    if(OPENVINO_LIB_PATH)
        get_filename_component(OPENVINO_LIB_DIR ${OPENVINO_LIB_PATH} DIRECTORY)
    else()
        set(OPENVINO_LIB_DIR "")
    endif()
else()
    # Fallback to pkg-config (optional)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(OPENVINO QUIET openvino)
    endif()
    if(OPENVINO_FOUND)
        message(STATUS "Found OpenVINO via pkg-config")
        message(STATUS "OpenVINO include dirs: ${OPENVINO_INCLUDE_DIRS}")
        message(STATUS "OpenVINO library dirs: ${OPENVINO_LIBRARY_DIRS}")
        set(OPENVINO_AVAILABLE TRUE)
        set(OPENVINO_LIB_DIR ${OPENVINO_LIBRARY_DIRS})
        # Create imported targets for OpenVINO libraries
        add_library(openvino::runtime SHARED IMPORTED)
        set_target_properties(openvino::runtime PROPERTIES
            IMPORTED_LOCATION "${OPENVINO_LIB_DIR}/libopenvino.so"
            INTERFACE_INCLUDE_DIRECTORIES "${OPENVINO_INCLUDE_DIRS}"
        )
        add_library(openvino::runtime::c SHARED IMPORTED)
        set_target_properties(openvino::runtime::c PROPERTIES
            IMPORTED_LOCATION "${OPENVINO_LIB_DIR}/libopenvino_c.so"
            INTERFACE_INCLUDE_DIRECTORIES "${OPENVINO_INCLUDE_DIRS}"
        )
        set(OPENVINO_LIBS openvino::runtime openvino::runtime::c)
    else()
        message(WARNING "OpenVINO not found -- openvino_infer_node will NOT be built")
    endif()
endif()

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# coordinate_extractor: standalone pixel-coordinate picker tool (C++17, OpenCV only)
add_executable(coordinate_extractor src/coordinate_extractor.cpp)
set_target_properties(coordinate_extractor PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_definitions(coordinate_extractor PRIVATE
    DEFAULT_TEMPLATE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/template"
    DEFAULT_RAW_IMAGE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/raw_image"
)
target_link_libraries(coordinate_extractor ${OpenCV_LIBS})
target_include_directories(coordinate_extractor PRIVATE ${OpenCV_INCLUDE_DIRS})

# ===== YOLO Model Variant Selection =====
# USE_YOLO26=ON : Use YOLO26 (NMS-Free) model
#colcon build --packages-select openvino_infer --cmake-args -DUSE_YOLO26=ON 
# USE_YOLO26=OFF: Use YOLO12 model (default)
option(USE_YOLO26 "Build with YOLO26 (NMS-Free pose) model instead of YOLO12" ON)
if(USE_YOLO26)
    add_definitions(-DUSE_YOLO26)
    message(STATUS ">>> YOLO26 (NMS-Free) model selected")
else()
    message(STATUS ">>> YOLO12 model selected (default)")
endif()

# openvino_infer_node: only built when OpenVINO is available
if(OPENVINO_AVAILABLE)
    # 动态库：支持 rclcpp_components 组件注册（进程内通信）
    add_library(openvino_infer_component SHARED src/openvino_infer_node.cpp)

    # Link libraries (modern CMake targets)
    target_link_libraries(openvino_infer_component PUBLIC
        ${OpenCV_LIBS}
        ${OPENVINO_LIBS}
        ${YAML_CPP_SHARED_LIB}
        ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
        cv_bridge::cv_bridge
        rclcpp::rclcpp
        rclcpp_components::component
        rclcpp_components::component_manager
        sensor_msgs::sensor_msgs_library
    )

    # Include directories
    target_include_directories(openvino_infer_component PRIVATE
        ${CMAKE_BINARY_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
    )

    # 注册组件，同时自动生成独立可执行文件（兼容 ros2 run 独立启动）
    rclcpp_components_register_node(openvino_infer_component
        PLUGIN "OpenVINOInferenceNode"
        EXECUTABLE openvino_infer_node
    )

    # Set RPATH for OpenVINO
    if(OPENVINO_LIB_DIR)
        set_target_properties(openvino_infer_component PROPERTIES
            BUILD_RPATH "${OPENVINO_LIB_DIR}"
            INSTALL_RPATH "${OPENVINO_LIB_DIR}"
        )
    endif()
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type (Release/Debug/...)" FORCE)

# Install rules
install(TARGETS coordinate_extractor
  DESTINATION lib/${PROJECT_NAME}
)
if(OPENVINO_AVAILABLE)
  install(TARGETS openvino_infer_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install model files (optional)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models)
  install(DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

# YOLO26 model files
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models26)
  install(DIRECTORY models26/
    DESTINATION share/${PROJECT_NAME}/models26
  )
endif()

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
