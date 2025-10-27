#include "processor.h"
#include "global_image_buffer.h"
#include <string.h>
#include <stddef.h>
#include "image.h"
#ifdef PROCESSOR_VERIFY_BINARY
#include <assert.h>
#endif

#ifndef RESTRICT
#if defined(__GNUC__) || defined(__clang__)
#define RESTRICT __restrict__
#elif defined(_MSC_VER)
#define RESTRICT __restrict
#else
#define RESTRICT
#endif
#endif

// 默认实现：按契约 original 已是 0/255；默认直接拷贝到 imo。
// 可选：定义 SANITIZE_INPUT 时，对非 0/255 的输入做阈值归一化。
void process_original_to_imo(const uint8_t * RESTRICT original,
                             uint8_t * RESTRICT imo_out,
                             int width,
                             int height) {
    if (width <= 0 || height <= 0) return;

    // 将输入 original 拷贝到全局 Grayscale（image_process 使用该缓冲作为输入）
    for (int y = 0; y < height; ++y) {
        memcpy(Grayscale[y], original + y * width, (size_t)width);
    }

    // 调用你的流水线
    image_process();

    // 若调用者传入的 imo_out 不是全局 imo，则把结果复制回去
    if (imo_out != &imo[0][0]) {
        for (int y = 0; y < height; ++y) {
            memcpy(imo_out + y * width, &imo[y][0], (size_t)width);
        }
    }
}


