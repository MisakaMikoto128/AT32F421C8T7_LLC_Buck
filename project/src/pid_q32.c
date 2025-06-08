/**
 * @file pid_q32.c
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2025-06-04
 * @last modified 2025-06-04
 *
 * @copyright Copyright (c) 2025 Liu Yuanlin Personal.
 *
 */
#include "pid_q32.h"

void Inc_PID_Q32_Init(pInc_PID_Q32_t self)
{
    self->iTarget   = 0; // 目标值
    self->iSampling = 0; // 采样值

    self->P = 0; // 比例
    self->I = 0; // 积分
    self->D = 0; // 微分

    self->iError     = 0; // 当前误差
    self->iPrevError = 0; // 前1次误差值
    self->iLastError = 0; // 前2次误差值

    self->iF    = 0; // 控制器输出值
    self->iFmax = 0; // 控制器输出最大值
    self->iFmin = 0; // 控制器输出最小值
}

/**
 * @brief Update the PID controller with the current target and sampling values,
 *        and add the delta to the output.
 *
 * @param self Pointer to the PID controller instance.
 * @return int Returns 1 if output was limited to max, -1 if limited to min, 0 otherwise.
 */
int Inc_PID_Q32_Update_AddDelta(pInc_PID_Q32_t self)
{
    int res       = 0;
    int32_t delta = 0;
    int64_t F     = 0;
    // Calculate current error
    self->iError = self->iTarget - self->iSampling;
    delta        = self->P * (self->iError - self->iLastError) +
            self->I * self->iError +
            self->D * (self->iError - self->iPrevError);

    // Calculate total output
    // Update the previous error
    self->iPrevError = self->iLastError;
    // Update last error
    self->iLastError = self->iError;

    F = self->iF + delta;

    // Restrict to max/min
    if (F >= self->iFmax) {
        self->iF = self->iFmax;
        res      = 1; // Indicate that the output was limited to max
    } else if (F <= self->iFmin) {
        self->iF = self->iFmin;
        res      = -1; // Indicate that the output was limited to min
    } else {
        self->iF = F;
    }
    return res; // Return the result of the update
}

/**
 * @brief Update the PID controller with the current target and sampling values,
 *        and subtract the delta from the output.
 *
 * @param self Pointer to the PID controller instance.
 * @return int Returns 1 if output was limited to max, -1 if limited to min, 0 otherwise.
 */
int Inc_PID_Q32_Update_SubDelta(pInc_PID_Q32_t self)
{

    int res       = 0;
    int32_t delta = 0;
    int64_t F     = 0;
    // Calculate current error
    self->iError  = self->iTarget - self->iSampling;
    delta         = self->P * (self->iError - self->iLastError) +
            self->I * self->iError +
            self->D * (self->iError - self->iPrevError);

    // Calculate total output
    // Update the previous error
    self->iPrevError = self->iLastError;
    // Update last error
    self->iLastError = self->iError;

    F = self->iF - delta;

    // Restrict to max/min
    if (F >= self->iFmax) {
        self->iF = self->iFmax;
        res      = 1; // Indicate that the output was limited to max
    } else if (F <= self->iFmin) {
        self->iF = self->iFmin;
        res      = -1; // Indicate that the output was limited to min
    } else {
        self->iF = F;
    }
    return res; // Return the result of the update
}