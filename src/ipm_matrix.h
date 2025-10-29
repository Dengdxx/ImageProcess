// 自动生成的 IPM 逆矩阵 M (H_inv)
// 用于在 STM32 启动时生成 RAM 查找表

#include <stdint.h>

// 图像尺寸宏定义
#define IPM_OUT_HEIGHT 120
#define IPM_OUT_WIDTH  188
#define IPM_SRC_HEIGHT 120
#define IPM_SRC_WIDTH  188
#define IPM_INVALID_MARKER_UINT8 255

// 逆单应性矩阵 M = H_inverse (存入 Flash)
// 这是 STM32 用来进行浮点运算的唯一数据
const float ipm_matrix[3][3] = {
  { 0.70257766f, -0.20146909f, 23.97482162f },
  { -0.00000000f, 0.50390975f, 9.58992865f },
  { -0.00000000f, -0.00215475f, 0.95899286f },
};
