/**
 * @file ccommon.h
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2024-04-17
 * @last modified 2024-04-17
 *
 * @copyright Copyright (c) 2024 Liu Yuanlin Personal.
 *
 */
#ifndef CCOMMON_H
#define CCOMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#define byte_t uint8_t

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(x)    ((x) < 0 ? -(x) : (x))

/* ===================== C89/C90 兼容的基础宏 ===================== */

// 字符串化宏
#define STRINGIFY(x) #x
#define TOSTRING(x)  STRINGIFY(x)

// 连接宏
#define CONCAT(a, b)     a##b
#define CONCAT3(a, b, c) a##b##c

/* 结构体成员偏移 */
#ifndef offsetof
#define offsetof(type, member) ((size_t)&((type *)0)->member)
#endif

// 获取数组大小
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

// 容器宏 - 从结构体成员指针获取容器指针
#define CONTAINER_OF(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))

/* ===================== 编译时断言（C89兼容） ===================== */

/* 方法1: 使用数组大小 */
#define STATIC_ASSERT_C89(condition, name) \
    typedef char static_assertion_##name[(condition) ? 1 : -1]

/* 方法2: 使用枚举 */
#define STATIC_ASSERT_ENUM(condition, name)           \
    enum {                                            \
        static_assertion_##name = 1 / (!!(condition)) \
    }

// 兼容 C11 之前的静态断言
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
// 如果支持 C11，直接使用 _Static_assert，C11 已经内置 _Static_assert
#define STATIC_ASSERT(COND, MSG) _Static_assert(COND, #MSG)
#else
#define STATIC_ASSERT(COND, MSG)   typedef char static_assertion_##MSG[(!!(COND)) * 2 - 1]
// token pasting madness:
#define COMPILE_TIME_ASSERT3(X, L) STATIC_ASSERT(X, static_assertion_at_line_##L)
#define COMPILE_TIME_ASSERT2(X, L) COMPILE_TIME_ASSERT3(X, L)
#define COMPILE_TIME_ASSERT(X)     COMPILE_TIME_ASSERT2(X, __LINE__)
#endif

/* ===================== 标准C的类型安全宏 ===================== */

/* 不使用typeof的安全交换宏 */
#define SWAP_INT(a, b)   \
    do {                 \
        int temp = (a);  \
        (a)      = (b);  \
        (b)      = temp; \
    } while (0)

#define SWAP_FLOAT(a, b)   \
    do {                   \
        float temp = (a);  \
        (a)        = (b);  \
        (b)        = temp; \
    } while (0)

#define SWAP_DOUBLE(a, b)   \
    do {                    \
        double temp = (a);  \
        (a)         = (b);  \
        (b)         = temp; \
    } while (0)

#define SWAP_PTR(a, b)     \
    do {                   \
        void *temp = (a);  \
        (a)        = (b);  \
        (b)        = temp; \
    } while (0)

/* ===================== 位操作宏 ===================== */

#define BIT_SET(value, bit)   ((value) |= (1UL << (bit)))
#define BIT_CLEAR(value, bit) ((value) &= ~(1UL << (bit)))
#define BIT_FLIP(value, bit)  ((value) ^= (1UL << (bit)))
#define BIT_CHECK(value, bit) (((value) >> (bit)) & 1UL)
#define BIT_MASK(bits)        ((1UL << (bits)) - 1UL)

/* 位字段操作 */
#define BITFIELD_GET(value, start, width) \
    (((value) >> (start)) & BIT_MASK(width))

#define BITFIELD_SET(value, start, width, new_val)         \
    ((value) = ((value) & ~(BIT_MASK(width) << (start))) | \
               (((new_val) & BIT_MASK(width)) << (start)))

#ifdef __cplusplus
}
#endif
#endif //! CCOMMON_H
