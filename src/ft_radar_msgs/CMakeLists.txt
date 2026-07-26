cmake_minimum_required(VERSION 3.8)
project(ft_radar_msgs)

# 纯消息包，无自定义 C++ 代码；rosidl 生成的 C++ 代码可能有编译警告，属正常现象
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# V2 架构消息类型:
#   - AdcFilePath / CameraFilePath: 文件路径发布模式 (替代 AdcRawData 的 32MB DDS 传输)
#   - SystemMonitor: 系统监控节点输出
#   - DetPoint / DetList: 雷达检测结果
#   - EgoMotion: 自车运动数据
#   - Object3D / ObjList: 3D 目标检测
# 已移除: AdcRawData.msg (被 AdcFilePath 替代), RnNciData.msg (未使用)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AdcFilePath.msg"
  "msg/CameraFilePath.msg"
  "msg/SystemMonitor.msg"
  "msg/DetPoint.msg"
  "msg/DetList.msg"
  "msg/Object3D.msg"
  "msg/ObjList.msg"
  "msg/EgoMotion.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
