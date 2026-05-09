cmake_minimum_required(VERSION 3.10)
project(armor_processor)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-g)

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Release)
# endif()

set(OpenVINO_LIBRARIES openvino::runtime)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

link_directories(/usr/lib/x86_64-linux-gnu)  # 替换为你的 fmt 库路径（含 .so 或 .a 文件）

find_package(fmt 8 REQUIRED)
find_package(CURL REQUIRED)
find_package(TIFF REQUIRED)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${YAML_CPP_INCLUDE_DIR}
)

ament_auto_find_build_dependencies()

include_directories(${PROJECT_SOURCE_DIR})
find_package(tools REQUIRED)
find_package(auto_aim REQUIRED)


###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC ${tf2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${tf2_LIBS} tools::tools auto_aim::auto_aim)
target_link_libraries(${PROJECT_NAME} ${OpenVINO_LIBRARIES})


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN auto_aim::ArmorProcessorNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  # USE_SCOPED_HEADER_INSTALL_DIR
)
