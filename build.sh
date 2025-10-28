#!/bin/bash
# 本脚本用于在Linux环境下配置和编译项目

# 设置脚本在任何命令失败时立即退出
set -e

echo "--- 正在配置 CMake... ---"
# -S . 指定源码目录为当前目录
# -B build 指定构建目录为 build
cmake -S . -B build

echo "--- 正在编译项目... ---"
# --build build 指定编译 build 目录下的项目
# 使用 -j 4 利用4个核心并行编译，可以加快速度
cmake --build build -- -j 4

echo "--- 编译完成！可执行文件位于 install/bin/ ---"
