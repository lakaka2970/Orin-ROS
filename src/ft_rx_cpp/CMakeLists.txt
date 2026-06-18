cmake_minimum_required(VERSION 3.8)
project(ft_rx_cpp)

# C++17 for std::if constexpr, structured bindings, <string_view>
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---- dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ft_radar_msgs REQUIRED)

# ---- include ----
include_directories(include)

# ---- common compile options ----
add_compile_options(-Wall -Wextra -Wpedantic)
# Release optimisation: -O2 is default for ament_cmake, explicit for safety
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O2)

# ============================================================================
# 1. vehicle_data_rx — lightweight, ~50 Hz scalar data (delivered first for CI)
# ============================================================================
add_executable(vehicle_data_rx_cpp src/vehicle_data_rx.cpp)
ament_target_dependencies(vehicle_data_rx_cpp
  rclcpp std_msgs geometry_msgs ft_radar_msgs)
target_include_directories(vehicle_data_rx_cpp PRIVATE include)

# ============================================================================
# 2. camera_rx — 1280×720 test-pattern, cv_bridge → sensor_msgs::Image
# ============================================================================
add_executable(camera_rx_cpp src/camera_rx.cpp)
ament_target_dependencies(camera_rx_cpp
  rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros cv_bridge OpenCV)
target_include_directories(camera_rx_cpp PRIVATE include)

# ============================================================================
# 3. adc_rx — 32 MB/frame zero-copy-optimised heavy-lift node
# ============================================================================
add_executable(adc_rx_cpp src/adc_rx.cpp)
ament_target_dependencies(adc_rx_cpp
  rclcpp std_msgs geometry_msgs tf2_ros ft_radar_msgs)
target_include_directories(adc_rx_cpp PRIVATE include)
# std::filesystem needs explicit linking on GCC 9 (Foxy)
target_link_libraries(adc_rx_cpp stdc++fs)

# ---- install ----
install(TARGETS
  vehicle_data_rx_cpp
  camera_rx_cpp
  adc_rx_cpp
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

ament_package()
