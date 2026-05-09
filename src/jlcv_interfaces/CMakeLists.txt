cmake_minimum_required(VERSION 3.8)
project(jlcv_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/DebugArmor.msg"
  "msg/DebugArmors.msg"
  "msg/DebugLight.msg"
  "msg/DebugLights.msg"
  "msg/EnergyTarget.msg"
  "msg/GameStatus.msg"
  "msg/Gimbal.msg"
  "msg/Measurement.msg"
  "msg/Target.msg"
  
  "srv/ResetColor.srv"
  "srv/SetMode.srv"

  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_package()
