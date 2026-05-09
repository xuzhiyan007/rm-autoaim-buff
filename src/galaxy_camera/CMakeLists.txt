cmake_minimum_required(VERSION 3.8)
project(galaxy_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 添加头文件 include
include_directories(include)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/GxCamera.cpp
)

# 添加.so文件  DaHengGxiSDK
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
	target_link_libraries(${PROJECT_NAME}
	  ${PROJECT_SOURCE_DIR}/lib/x86/libgxiapi.so
	  ${OpenCV_LIBS}
	)
	install(
	  DIRECTORY lib/x86/      	      
	  DESTINATION lib/${PROJECT_NAME}    # 拷贝到install目录中
	)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
	target_link_libraries(${PROJECT_NAME}
	  ${PROJECT_SOURCE_DIR}/lib/arm/libgxiapi.so
	  ${OpenCV_LIBS}
	)
	install(
	  DIRECTORY lib/arm/      
	  DESTINATION lib/${PROJECT_NAME}    
	)
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN galaxy_camera::GxCamera
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

# ############
# # Install ##
# ############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
