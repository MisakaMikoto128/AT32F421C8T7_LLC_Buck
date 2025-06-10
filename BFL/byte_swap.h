/**
 * @file byte_swap.h
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2024-11-20
 * @last modified 2024-11-20
 *
 * @copyright Copyright (c) 2024 Liu Yuanlin Personal.
 *
 */
#ifndef BYTE_SWAP_H
#define BYTE_SWAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// 检测是否支持 __REV16 指令
#if defined(__GNUC__) || defined(__ARMCC_VERSION) || defined(_MSC_VER)
#if defined(__ARM_ARCH) || defined(__ARM_FEATURE_DSP) || defined(__ARM_ARCH_ISA_THUMB)
#include "cmsis_armclang.h"
#endif
#endif

#ifndef __REV16
#define __REV16(value) (((value & 0xFF00) >> 8) | ((value & 0x00FF) << 8))
#endif // !__REV16
#ifdef __cplusplus
}
#endif
#endif //! BYTE_SWAP_H
