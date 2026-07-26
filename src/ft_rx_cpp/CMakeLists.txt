cmake_minimum_required(VERSION 3.8)
project(ft_rx_cpp)

# C++17 for std::filesystem, structured bindings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- dependencies (V2: 移除 OpenCV/cv_bridge, camera_rx 改为 Raw V4L2) ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ft_radar_msgs REQUIRED)

# ---- include ----
include_directories(include)

# ---- common compile options ----
add_compile_options(-Wall -Wextra -Wpedantic)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O2)
endif()

# ============================================================================
# 1. adc_rx — V4L2 双设备采集 + 文件路径发布 + 内置 Logging
# ============================================================================
add_executable(adc_rx_cpp src/adc_rx.cpp)
ament_target_dependencies(adc_rx_cpp
  rclcpp std_msgs geometry_msgs tf2_ros ft_radar_msgs)
target_include_directories(adc_rx_cpp PRIVATE include)
target_link_libraries(adc_rx_cpp stdc++fs)

# ============================================================================
# 2. camera_rx — Raw V4L2 mmap + MJPEG 直接写盘 + 文件路径发布
# ============================================================================
add_executable(camera_rx_cpp src/camera_rx.cpp)
ament_target_dependencies(camera_rx_cpp
  rclcpp std_msgs geometry_msgs tf2_ros ft_radar_msgs)
target_include_directories(camera_rx_cpp PRIVATE include)
target_link_libraries(camera_rx_cpp stdc++fs)

# ============================================================================
# 3. vehicle_data_rx — SocketCAN + 硬件时间戳 + 内置 Ego Logging
# ============================================================================
add_executable(vehicle_data_rx_cpp src/vehicle_data_rx.cpp)
ament_target_dependencies(vehicle_data_rx_cpp
  rclcpp std_msgs geometry_msgs ft_radar_msgs)
target_include_directories(vehicle_data_rx_cpp PRIVATE include)
target_link_libraries(vehicle_data_rx_cpp stdc++fs)

# ---- install ----
install(TARGETS
  adc_rx_cpp
  camera_rx_cpp
  vehicle_data_rx_cpp
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
