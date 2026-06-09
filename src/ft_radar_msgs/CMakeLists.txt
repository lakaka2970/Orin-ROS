cmake_minimum_required(VERSION 3.8)
project(ft_radar_msgs)

# 默认 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 生成所有自定义消息类型
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdcRawData.msg"
  "msg/DetPoint.msg"
  "msg/DetList.msg"
  "msg/Object3D.msg"
  "msg/ObjList.msg"
  "msg/EgoMotion.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
