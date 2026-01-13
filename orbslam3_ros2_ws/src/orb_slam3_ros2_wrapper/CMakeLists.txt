cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros2_wrapper LANGUAGES CXX)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. RPATH 全局设置 (关键修复) ---
# 这段代码确保安装后的程序既能找到 ORB_SLAM3，也能找到本项目生成的 common 库

# 跳过构建时的 RPATH 处理 (使用默认)
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
# 安装时重写 RPATH
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
# 自动将链接的库路径添加到 RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 定义需要加入 RPATH 的路径列表
# 1. 外部库路径 (ORB_SLAM3)
# 2. 本项目库的安装路径 (${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
list(APPEND CMAKE_INSTALL_RPATH 
    "${ORB_SLAM3_ROOT_DIR}/lib"
    "${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib"
    "${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib"
    "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}" 
)

# --- 2. 查找依赖 ---
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(slam_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

option(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA "Enable CUDA support if available" OFF)

if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  include(CheckLanguage)
  enable_language(CUDA)
  message(WARNING "CUDA support requested for ORB_SLAM3_ROS2_WRAPPER")
endif()

# --- 3. 包含路径 ---
include_directories(
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
endif()

set(ros_dependencies 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  message_filters 
  tf2_ros 
  tf2_eigen 
  slam_msgs 
  pcl_ros 
  pcl_conversions 
  nav_msgs 
  std_srvs
)

# --- 4. 编译公共库 ---
add_library(orb_slam3_ros2_wrapper_common SHARED
  src/type_conversion.cpp
  src/orb_slam3_interface.cpp
  src/time_profiler.cpp
  src/slam_node_base.cpp
)
ament_target_dependencies(orb_slam3_ros2_wrapper_common ${ros_dependencies})

# 链接库
target_link_libraries(orb_slam3_ros2_wrapper_common 
  ${ORB_SLAM3_LIBRARIES} 
  ${Pangolin_LIBRARIES} 
  ${Sophus_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  target_compile_definitions(orb_slam3_ros2_wrapper_common PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()

# --- 5. 编译节点 ---
# RGBD
add_executable(rgbd src/rgbd/rgbd-slam-node.cpp src/rgbd/rgbd.cpp)
ament_target_dependencies(rgbd ${ros_dependencies})
target_link_libraries(rgbd orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(rgbd PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(rgbd PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()

# RGBD-IMU
add_executable(rgbd_imu src/rgbd-imu/rgbd-imu-slam-node.cpp src/rgbd-imu/rgbd_imu.cpp)
ament_target_dependencies(rgbd_imu ${ros_dependencies})
target_link_libraries(rgbd_imu orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(rgbd_imu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(rgbd_imu PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()

# Mono-IMU
add_executable(mono_imu src/mono-imu/mono-imu-slam-node.cpp src/mono-imu/mono_imu.cpp)
ament_target_dependencies(mono_imu ${ros_dependencies})
target_link_libraries(mono_imu orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(mono_imu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(mono_imu PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()

# Stereo
add_executable(stereo src/stereo/stereo-slam-node.cpp src/stereo/stereo.cpp)
ament_target_dependencies(stereo ${ros_dependencies})
target_link_libraries(stereo orb_slam3_ros2_wrapper_common ${PCL_LIBRARIES})
if(ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA)
  set_target_properties(stereo PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
  target_compile_definitions(stereo PRIVATE ORB_SLAM3_ROS2_WRAPPER_ENABLE_CUDA=1)
endif()

# --- 6. 安装 ---
# 注意：这里我们把库和可执行文件都安装到了 lib/${PROJECT_NAME} 下
install(TARGETS orb_slam3_ros2_wrapper_common rgbd rgbd_imu mono_imu stereo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(gtest)
  find_package(ament_cmake_gtest REQUIRED)
  ament_find_gtest()
  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})
  
  ament_add_gtest(typeconversionTests tests/typeConversionsTests.cpp src/type_conversion.cpp)
  ament_target_dependencies(typeconversionTests rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin tf2_ros tf2_eigen slam_msgs pcl_ros pcl_conversions PCL nav_msgs)
endif()

ament_package()