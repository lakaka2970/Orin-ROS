cmake_minimum_required(VERSION 3.8)
project(ft_radar_msgs)

# 纯消息包，无自定义 C++ 代码；rosidl 生成的 C++ 代码可能有编译警告，属正常现象
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
  "msg/RnNciData.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
