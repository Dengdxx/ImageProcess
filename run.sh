#!/bin/bash
# 本脚本用于在Linux环境下运行 imageprocessor 程序

set -e

EXECUTABLE_PATH="./install/bin/imageprocessor"

# 1. 检查可执行文件是否存在
if [ ! -f "$EXECUTABLE_PATH" ]; then
    echo "错误: 在 $EXECUTABLE_PATH 未找到可执行文件。"
    echo "请先运行 ./build.sh 进行编译。"
    exit 1
fi

# 2. 运行程序, 并将所有脚本参数传递给程序
echo "--- 正在启动 ImageProcessor ---"
"$EXECUTABLE_PATH" "$@"
