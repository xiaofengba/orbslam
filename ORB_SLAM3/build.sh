#!/bin/bash

# 遇到错误立即停止 (非常重要，防止前面编译失败后面继续跑)
set -e

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
# --- 修改点：彻底清理旧文件 ---
rm -rf build lib
# ---------------------------
mkdir build
cd build
# 建议改为 C++17 以适配 ROS 2 Humble
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++17
make -j6
# make install # 注意：原生 ORB_SLAM3 的 DBoW2 通常不需要 make install，它会把库放在 ../lib 下。如果你确定需要就取消注释

cd ../../g2o 

echo "Configuring and building Thirdparty/g2o ..."

# --- 修改点：彻底清理旧文件 ---
rm -rf build lib
# ---------------------------
mkdir build
cd build
# 建议改为 C++17
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++17
make -j6
# make install 

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

# --- 修改点：彻底清理旧文件 ---
rm -rf build lib
# ---------------------------
mkdir build
cd build
# 建议改为 C++17
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++17
make -j6
# make install

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
if [ ! -f "ORBvoc.txt" ]; then
    tar -xf ORBvoc.txt.tar.gz
fi
cd ..

echo "Configuring and building ORB_SLAM3 ..."

# --- 修改点：彻底清理旧文件 ---
rm -rf build lib
# ---------------------------
mkdir build
cd build
# 建议改为 C++17
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++17
make -j6