#include <stdint.h>
#include <math.h>   // for roundf (编译时需链接 -lm)

// 1. 包含你用 Python 生成的矩阵头文件
//    (这个 .h 文件定义了所有 IPM_* 宏和 ipm_matrix)
#include "ipm_matrix.h" 

/*
 * -------------------------------------------------------------
 * 1. 核心 RAM 查找表 (LUT) 缓冲区 (!!已修改!!)
 * -------------------------------------------------------------
 * 职责：存储高精度的定点坐标 (Q8.8 格式)
 * (高 8 位 = 整数, 低 8 位 = 小数)
 * (RAM 占用已翻倍: 2 * 120 * 188 * 2(uint16_t) = 90,240 字节)
 */
uint16_t g_ipm_lut_x_fx[IPM_OUT_HEIGHT][IPM_OUT_WIDTH];
uint16_t g_ipm_lut_y_fx[IPM_OUT_HEIGHT][IPM_OUT_WIDTH];

// 定义定点数的小数位数 (8 位 = 256)
#define FX_SHIFT 8
#define FX_ONE   (1 << FX_SHIFT) // 1.0
#define FX_MASK  (FX_ONE - 1)    // 0.996... (即 0xFF)


/*
 * -------------------------------------------------------------
 * 2. 核心函数 1: 初始化 (函数名与旧版一致)
 * -------------------------------------------------------------
 * 职责：(新) 使用 FPU，在 RAM 中生成 *高精度* (Q8.8) 查找表。
 */
void generate_ipm_lut_in_ram(void) {
    
    for (int v_dst = 0; v_dst < IPM_OUT_HEIGHT; v_dst++) {
        for (int u_dst = 0; u_dst < IPM_OUT_WIDTH; u_dst++) {
            
            const float u = (float)u_dst;
            const float v = (float)v_dst;

            const float w_prime = ipm_matrix[2][0] * u + ipm_matrix[2][1] * v + ipm_matrix[2][2];
            
            float x_src_f = -1.0f; // 无效标记
            float y_src_f = -1.0f; // 无效标记
            
            if (w_prime != 0.0f) {
                x_src_f = (ipm_matrix[0][0] * u + ipm_matrix[0][1] * v + ipm_matrix[0][2]) / w_prime;
                y_src_f = (ipm_matrix[1][0] * u + ipm_matrix[1][1] * v + ipm_matrix[1][2]) / w_prime;
            }
            
            // --- 边界检查 (重要!) ---
            // 检查 x/y 是否在 [0, width-1] 和 [0, height-1] 之间
            if (x_src_f < 0.0f || x_src_f > (IPM_SRC_WIDTH - 1.0f) ||
                y_src_f < 0.0f || y_src_f > (IPM_SRC_HEIGHT - 1.0f)) {
                
                // 标记为无效 (使用 0xFFFF)
                g_ipm_lut_x_fx[v_dst][u_dst] = 0xFFFF; 
                g_ipm_lut_y_fx[v_dst][u_dst] = 0xFFFF;
            } 
            else {
                // (新) 转换为 Q8.8 定点数
                uint16_t x_fx = (uint16_t)(x_src_f * FX_ONE); 
                uint16_t y_fx = (uint16_t)(y_src_f * FX_ONE);
            
                // 存入 RAM 查找表
                g_ipm_lut_x_fx[v_dst][u_dst] = x_fx;
                g_ipm_lut_y_fx[v_dst][u_dst] = y_fx;
            }
        }
    }
}


/*
 * -------------------------------------------------------------
 * 3. 核心函数 2: 实时变换 (!!已修改为 1D 接口!!)
 * -------------------------------------------------------------
 * 职责：使用 RAM-LUT 和双线性插值进行高速变换。
 * (接口已改为 uint8_t* 来匹配你的 processor.c)
 */
void fast_ipm_transform(const uint8_t* input, 
                              uint8_t* output) 
{
    // (generate_ipm_lut_in_ram 函数保持不变)

    for (int v = 0; v < IPM_OUT_HEIGHT; v++) {
        for (int u = 0; u < IPM_OUT_WIDTH; u++) {
            
            // 1. 从 RAM 中查找高精度坐标
            const uint16_t x_fx = g_ipm_lut_x_fx[v][u];
            const uint16_t y_fx = g_ipm_lut_y_fx[v][u];

            // 2. (新) 计算输出图像的一维索引
            const int dst_idx = v * IPM_OUT_WIDTH + u;

            // 3. 检查无效标记 (0xFFFF)
            if (x_fx == 0xFFFF) {
                output[dst_idx] = 0; // 填充黑色
            } 
            else {
                // 4. (新) 解码 Q8.8 定点数
                const int x0 = x_fx >> FX_SHIFT; // 整数 x
                const int y0 = y_fx >> FX_SHIFT; // 整数 y
                
                const int x1 = x0 + 1; // (边界已在 init 时检查过)
                const int y1 = y0 + 1; // (边界已在 init 时检查过)

                // 提取小数 (权重)
                const int wx = x_fx & FX_MASK; // 权重 x (0-255)
                const int wy = y_fx & FX_MASK; // 权重 y (0-255)
                const int wx_inv = FX_ONE - wx; // 1.0 - wx
                const int wy_inv = FX_ONE - wy; // 1.0 - wy

                // 5. (新) 计算 4 个相邻像素的一维索引
                const int p00_idx = y0 * IPM_SRC_WIDTH + x0;
                const int p01_idx = y0 * IPM_SRC_WIDTH + x1;
                const int p10_idx = y1 * IPM_SRC_WIDTH + x0;
                const int p11_idx = y1 * IPM_SRC_WIDTH + x1;

                // 6. (新) 读取 4 个像素
                const int p00 = input[p00_idx];
                const int p01 = input[p01_idx];
                const int p10 = input[p10_idx];
                const int p11 = input[p11_idx];
                
                // 7. (新) 执行双线性插值
                int top_interp = (p00 * wx_inv + p01 * wx); 
                int bot_interp = (p10 * wx_inv + p11 * wx);
                int final_val = (top_interp * wy_inv + bot_interp * wy);

                // 8. (新) 归一化
                output[dst_idx] = (uint8_t)(final_val >> (FX_SHIFT * 2));
            }
        }
    }
}