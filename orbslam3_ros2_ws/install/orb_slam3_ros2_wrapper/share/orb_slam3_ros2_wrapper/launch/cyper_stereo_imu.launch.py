#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    # Essential_paths
    orb_wrapper_pkg = get_package_share_directory('orb_slam3_ros2_wrapper')

    # 1. 定义 LaunchConfiguration 占位符
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_namespace = LaunchConfiguration('robot_namespace')
    params_file = LaunchConfiguration('params_file') # 放到外面

    # 2. 定义 Arguments (这些必须在 OpaqueFunction 之前被 return)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace', 
        default_value="robot",
        description='The namespace of the robot')
    
    # [修复] 把它移到这里！
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(orb_wrapper_pkg, 'params', 'ros_params', 'cyper-stereo-imu-ros-params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 3. 启动逻辑
    def all_nodes_launch(context, robot_namespace):
        # 调试打印：现在这里能正确获取到路径了
        print("Param file path: ", params_file.perform(context))

        # 硬编码路径建议改为参数传入，或者检查路径是否正确
        vocabulary_file_path = "/home/sf/workspace/orb_slam3_second_ws/orbslam/ORB_SLAM3/Vocabulary/ORBvoc.txt"
        config_file_path = "/home/sf/workspace/orb_slam3_second_ws/orbslam/orbslam3_ros2_ws/src/orb_slam3_ros2_wrapper/params/orb_slam3_params/cyper_stereo_imu.yaml"

        param_substitutions = {}

        # 配置 RewrittenYaml
        configured_params = RewrittenYaml(
            source_file=params_file, # 这里现在是有效的
            root_key=robot_namespace.perform(context),
            param_rewrites=param_substitutions,
            convert_types=True)
        
        orb_slam3_node = Node(
            package="orb_slam3_ros2_wrapper",
            executable="stereo_imu",
            output="screen",
            namespace=robot_namespace.perform(context),
            # arguments 是传给 main() 函数的 argv，通常是 ORB-SLAM3 的词汇表和配置文件
            arguments=[vocabulary_file_path, config_file_path],
            # parameters 是 ROS 2 的节点参数，来自 yaml
            parameters=[configured_params] 
        )
        
        # 注意：这里只返回 Node，不要再返回 declare_params_file_cmd 了
        return [orb_slam3_node]

    opaque_function = OpaqueFunction(function=all_nodes_launch, args=[robot_namespace])

    # 4. 返回 LaunchDescription
    return LaunchDescription([
        declare_use_sim_time_cmd,
        robot_namespace_arg,
        declare_params_file_cmd, # [修复] 确保它在这里
        opaque_function
    ])