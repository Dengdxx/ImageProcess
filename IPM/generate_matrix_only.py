import cv2
import numpy as np

# -----------------------------------------------------------------
# 1. 最终配置 (必须与 STM32 侧匹配)
# -----------------------------------------------------------------
SRC_HEIGHT = 120 # 源图像高
SRC_WIDTH  = 188 # 源图像宽
OUT_HEIGHT = 120  # 目标鸟瞰图高
OUT_WIDTH  = 188 # 目标鸟瞰图宽

# 输出的头文件名
OUTPUT_H_FILE = "../src/ipm_matrix.h"

# -----------------------------------------------------------------
# 2. !!! 你的核心任务：标定 !!!
# -----------------------------------------------------------------
# 你必须在这里填入你从 120x188 摄像头原图中
# 找到的4个点 (x, y)。这4个点在现实中必须是一个矩形。
# 
# (x, y) 格式, (0,0) 在左上角
# -----------------------------------------------------------------
# (!!! 以下是示例值，你必须替换成你自己的 !!!)
src_points = np.float32([
    [25, 10],    # 源: 远处的左上角 (x, y)
    [162, 10],   # 源: 远处的右上角 (x, y)
    [187, 99],  # 源: 近处的右下角 (x, y)
    [0, 99]     # 源: 近处的左下角 (x, y)
])

# -----------------------------------------------------------------
# 3. 目标点 (固定为矩形，不需要修改)
# -----------------------------------------------------------------
# 我们强制将源梯形映射到 80x188 的完整矩形
dst_points = np.float32([
    [0, 0],                     # 目: 左上 (0, 0)
    [OUT_WIDTH - 1, 0],         # 目: 右上 (187, 0)
    [OUT_WIDTH - 1, OUT_HEIGHT - 1], # 目: 右下 (187, 79)
    [0, OUT_HEIGHT - 1]         # 目: 左下 (0, 79)
])

# -----------------------------------------------------------------
# 4. 计算与生成 (不需要修改)
# -----------------------------------------------------------------
print("开始标定...")
print(f"源尺寸: {SRC_WIDTH}x{SRC_HEIGHT}")
print(f"目标尺寸: {OUT_WIDTH}x{OUT_HEIGHT}")

print("正在计算逆矩阵 H_inv (即 M)...")
H = cv2.getPerspectiveTransform(src_points, dst_points)
H_inv = np.linalg.inv(H) # 这就是 STM32 需要的矩阵 M

print(f"逆矩阵 M (H_inv):\n{H_inv}\n")

print(f"正在将矩阵写入 {OUTPUT_H_FILE}...")
with open(OUTPUT_H_FILE, 'w') as f:
    f.write(f"// 自动生成的 IPM 逆矩阵 M (H_inv)\n")
    f.write(f"// 用于在 STM32 启动时生成 RAM 查找表\n\n")
    f.write("#include <stdint.h>\n\n")
    f.write(f"// 图像尺寸宏定义\n")
    f.write(f"#define IPM_OUT_HEIGHT {OUT_HEIGHT}\n")
    f.write(f"#define IPM_OUT_WIDTH  {OUT_WIDTH}\n")
    f.write(f"#define IPM_SRC_HEIGHT {SRC_HEIGHT}\n")
    f.write(f"#define IPM_SRC_WIDTH  {SRC_WIDTH}\n")
    f.write(f"#define IPM_INVALID_MARKER_UINT8 255\n\n")

    f.write(f"// 逆单应性矩阵 M = H_inverse (存入 Flash)\n")
    f.write(f"// 这是 STM32 用来进行浮点运算的唯一数据\n")
    f.write(f"const float ipm_matrix[3][3] = {{\n")
    for row in H_inv:
        # 格式化输出为 C 语言的 float 常量
        f.write(f"  {{ {row[0]:.8f}f, {row[1]:.8f}f, {row[2]:.8f}f }},\n")
    f.write("};\n")

print(f"成功! {OUTPUT_H_FILE} 已生成。")
print("请将此文件复制到你的 STM32 项目中。")