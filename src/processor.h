// 用户自定义 original->imo 处理函数声明
// 约定：
// - original: 输入的二值图像，值域严格使用 0(黑) 或 255(白)
// - imo: 输出数组，0=黑、255=白；1/2/3/4/5 等保留作标注颜色索引
// - width/height: 图像尺寸，当前为 188x120（可按需通用）
#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <stdint.h>

#include "global_image_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

void process_original_to_imo(const uint8_t *original,
                             uint8_t *imo_out,
                             int width,
                             int height);

#ifdef __cplusplus
}
#endif

#endif // PROCESSOR_H
