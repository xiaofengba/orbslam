# 仓库的说明
1. 参考链接：[https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git](https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git)
2. 注意OPENCV的版本，以及链接情况，否则有时不能正常运行，确保下面的两个链接的位置一致，我这里统一都在找4.5.5的opencv版本，可以按照自己的需求改
    ```
    find_package(OpenCV 4.5.4 EXACT REQUIRED)
    ```
3. 如果运行出错记得检查`opencv的版本`
    包装器的opencv版本
    ```
    ldd install/orb_slam3_ros2_wrapper/lib/orb_slam3_ros2_wrapper/stereo | grep opencv_core
    ```
    ORBSLAM3的opencv版本
    ```
    ldd /home/ubuntu/workspaces/orbslam/ORB_SLAM3/lib/libORB_SLAM3.so | grep opencv_core
    ```

# ORB_SLAM3 核心库编译
1. 流程
    ```
    # 最好重新clone仓库
    git clone https://github.com/suchetanrs/ORB_SLAM3.git
    
    # 进入 ORB_SLAM3 的文件夹中
    cd ORB_SLAM3

    # 执行完整的重置编译
    chmod +x build.sh
    ./build.sh

    # 后续改代码可以进行小批量编译
    ./build_small.sh
    ```
# ORB的 wrapper 编译
1. 修改 ORB_SLAM3 的路径
    ```
    cd orbslam3_ros2_ws
    # 打开文件后记得在里面修改路径
    nano src/orb_slam3_ros2_wrapper/CMakeModules/FindORB_SLAM3.cmake
    ```
2. 执行编译
    ```
    cd orbslam3_ros2_ws
    colcon build
    ```

# 运行双目的案例
1. 启动相机
    ```sh
    ros2 launch realsense2_camera rs_launch.py \
        enable_infra1:='true' \
        enable_infra2:='true' \
        enable_depth:='true' \
        enable_gyro:='true' \
        enable_accel:='true' \
        depth_module.profile:='640x480x15' \
        depth_module.emitter_enabled:=0 \
        depth_module.laser_power:='0.0'\
        unite_imu_method:=2

    ros2 param set /camera/camera depth_module.emitter_enabled 0
    ros2 param set /camera/camera depth_module.exposure 10000
    ```
2. 运行ORBSLAM3
    ```
    cd orbslam3_ros2_ws
    source install/setup.bash
    ros2 launch  orb_slam3_ros2_wrapper d435i_stereo.launch.py
    ```