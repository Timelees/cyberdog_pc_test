#pragma once

typedef unsigned char uint8x16_t __attribute__((vector_size(16)));
typedef signed char int8x16_t __attribute__((vector_size(16)));
typedef unsigned short uint16x8_t __attribute__((vector_size(16)));
typedef signed short int16x8_t __attribute__((vector_size(16)));
typedef unsigned int uint32x4_t __attribute__((vector_size(16)));
typedef int int32x4_t __attribute__((vector_size(16)));

#define vld1q_u8(ptr) ((uint8x16_t){})
#define vld1q_s8(ptr) ((int8x16_t){})
#define vmaxq_u8(a, b) ((uint8x16_t){})
#define vmaxq_s8(a, b) ((int8x16_t){})
