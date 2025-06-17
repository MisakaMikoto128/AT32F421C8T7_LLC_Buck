/**
* @file BFL_MEasure.h
* @author Liu Yuanlin (liuyuanlins@outlook.com)
* @brief
* @version 0.1
* @date 2025-06-17
* @last modified 2025-06-17
*
* @copyright Copyright (c) 2025 Liu Yuanlin Personal.
*
*/
#ifndef BFL_MEASURE_H
#define BFL_MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define VREF                     (3.3f)                               // 根据实际电压修改
#define ADC_MAX_VALUE            (4095)                               // ADC最大值
#define VOLTAGE_SCALE_FACTOR_Q15 (uint32_t)((VREF / 4096.0f) * 32768) // Q15格式

#ifdef __cplusplus
}
#endif
#endif //!BFL_MEASURE_H
