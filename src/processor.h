#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <stdint.h>

#ifndef RESTRICT
#if defined(__GNUC__) || defined(__clang__)
#define RESTRICT __restrict__
#elif defined(_MSC_VER)
#define RESTRICT __restrict
#else
#define RESTRICT
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

// The main image processing pipeline function
void process_original_to_imo(const uint8_t * RESTRICT original,
                             uint8_t * RESTRICT imo_out,
                             int width,
                             int height);

#ifdef __cplusplus
}
#endif

#endif // PROCESSOR_H