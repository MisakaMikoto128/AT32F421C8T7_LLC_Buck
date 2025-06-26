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

    // delta 限制初始化为int32_t最大/最小值
    self->maxDelta = INT32_MAX; // PID增量限制
    self->minDelta = INT32_MIN; // PID减量限制
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

    if (delta > self->maxDelta) {
        delta = self->maxDelta;
    } else if (delta < self->minDelta) {
        delta = self->minDelta;
    }

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
    self->iError = self->iTarget - self->iSampling;
    delta        = self->P * (self->iError - self->iLastError) +
            self->I * self->iError +
            self->D * (self->iError - self->iPrevError);

    // Calculate total output
    // Update the previous error
    self->iPrevError = self->iLastError;
    // Update last error
    self->iLastError = self->iError;

    if (delta > self->maxDelta) {
        delta = self->maxDelta;
    } else if (delta < self->minDelta) {
        delta = self->minDelta;
    }

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

void Inc_PID_Q32_Set_DeltaLimit(pInc_PID_Q32_t self, int32_t maxDelta, int32_t minDelta)
{
    self->maxDelta = maxDelta;
    self->minDelta = minDelta;
}

/**
 * @brief Reset the incremental PID controller states to zero
 *
 * @param self Pointer to the incremental PID controller structure
 * @return int Reset status (implementation defined)
 *
 * @details
 * This function resets the error states of the PID controller:
 * - Current error (iError)
 * - Previous error (iPrevError)
 * - Last error (iLastError)
 * All values are set to 0, effectively clearing the controller's memory.
 */
int Inc_PID_Q32_Reset(pInc_PID_Q32_t self)
{
    self->iError     = 0;
    self->iPrevError = 0;
    self->iLastError = 0;

    return 0;
}

// Position PID controller initialization
void Pos_PID_Q32_Init(pPos_PID_Q32_t self)
{
    self->iTarget   = 0; // 目标值
    self->iSampling = 0; // 采样值

    self->P = 0; // 比例
    self->I = 0; // 积分
    self->D = 0; // 微分

    self->iError     = 0; // 当前误差
    self->iPrevError = 0; // 前1次误差值
    self->iLastError = 0; // 前2次误差值

    self->iIntegral   = 0; // 积分值
    self->iDifference = 0; // 微分值

    self->iF    = 0; // 控制器输出值
    self->iFmax = 0; // 控制器输出最大值
    self->iFmin = 0; // 控制器输出最小值

    self->maxIntegral = INT32_MAX; // 积分上限
    self->minIntegral = INT32_MIN; // 积分下限
}

/**
 * @brief Update the position PID controller with the current target and sampling values,
 *        and add the delta to the output.
 *
 * @param self Pointer to the position PID controller instance.
 * @return int Returns 1 if output was limited to max, -1 if limited to min, 0 otherwise.
 */
int Pos_PID_Q32_Update_AddDelta(pPos_PID_Q32_t self)
{

    int res = 0;
    // Calculate current error
    self->iError = self->iTarget - self->iSampling;

    // Calculate the integral term
    self->iIntegral += self->I * self->iError;

    // Restrict to max/min
    if (self->iIntegral >= self->maxIntegral) {
        self->iIntegral = self->maxIntegral;
    } else if (self->iIntegral <= self->minIntegral) {
        self->iIntegral = self->minIntegral;
    }

    // Calculate the difference term
    self->iDifference = ((self->iDifference * 7) + (self->iError - self->iPrevError)) >> 3;

    // Calculate PID controller output
    self->iF = self->P * self->iError +
               self->iIntegral +
               self->D * self->iDifference;

    // Calculate total output
    // Update the previous error
    self->iPrevError = self->iLastError;
    // Update last error
    self->iLastError = self->iError;

    if (self->iF > self->iFmax) {
        self->iF = self->iFmax;
        res      = 1; // Indicate that the output was limited to max
    } else if (self->iF < self->iFmin) {
        self->iF = self->iFmin;
        res      = -1; // Indicate that the output was limited to min
    }

    return res; // Return the result of the update
}

/**
 * @brief Update the position PID controller with the current target and sampling values,
 *        and subtract the delta from the output.
 *
 * @param self Pointer to the position PID controller instance.
 * @return int Returns 1 if output was limited to max, -1 if limited to min, 0 otherwise.
 */
int Pos_PID_Q32_Update_SubDelta(pPos_PID_Q32_t self)
{

    int res = 0;
    // Calculate current error
    self->iError = self->iTarget - self->iSampling;

    // Calculate the integral term
    self->iIntegral += self->I * self->iError;

    // Restrict to max/min
    if (self->iIntegral >= self->maxIntegral) {
        self->iIntegral = self->maxIntegral;
    } else if (self->iIntegral <= self->minIntegral) {
        self->iIntegral = self->minIntegral;
    }

    // Calculate the difference term
    self->iDifference = ((self->iDifference * 7) + (self->iError - self->iPrevError)) >> 3;

    // Calculate PID controller output
    self->iF = self->P * self->iError +
               self->iIntegral +
               self->D * self->iDifference;

    // Calculate total output
    // Update the previous error
    self->iPrevError = self->iLastError;
    // Update last error
    self->iLastError = self->iError;

    if (self->iF > self->iFmax) {
        self->iF = self->iFmax;
        res      = 1; // Indicate that the output was limited to max
    } else if (self->iF < self->iFmin) {
        self->iF = self->iFmin;
        res      = -1; // Indicate that the output was limited to min
    }

    return res; // Return the result of the update
}

void Pos_PID_Q32_Set_IntegralLimit(pPos_PID_Q32_t self, int32_t maxIntegral, int32_t minIntegral)
{
    self->maxIntegral = maxIntegral;
    self->minIntegral = minIntegral;
}

int Pos_PID_Q32_Reset(pPos_PID_Q32_t self)
{
    self->iError     = 0;
    self->iPrevError = 0;
    self->iLastError = 0;

    self->iIntegral   = 0;
    self->iDifference = 0;

    return 0;
}