/**
* @file pid_fp32.h
* @author Liu Yuanlin (liuyuanlins@outlook.com)
* @brief
* @version 0.1
* @date 2025-06-08
* @last modified 2025-06-08
*
* @copyright Copyright (c) 2025 Liu Yuanlin Personal.
*
*/
#ifndef PID_FP32_H
#define PID_FP32_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float iTarget;   // 目标值
    float iSampling; // 测量值
    float P;         // 比例
    float I;         // 积分
    float D;         // 微分

    float iError;     // 当前误差
    float iPrevError; // 前1次误差值
    float iLastError; // 前2次误差值

    float iF;    // 传输给控制器的新控制值
    float iFmax; // 传输给控制器的最大控制值
    float iFmin; // 传输给控制器的最小控制值
} Inc_PID_FP32_t, *pInc_PID_FP32_t;

void Inc_PID_FP32_Init(pInc_PID_FP32_t self);
void Inc_PID_FP32_Update_AddDelta(pInc_PID_FP32_t self);
void Inc_PID_FP32_Update_SubDelta(pInc_PID_FP32_t self);
#ifdef __cplusplus
}
#endif
#endif //!PID_FP32_H
