/**
 * @file HDL_CRC.h
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief AT32硬件CRC外设实现CRC16算法接口
 * @version 0.1
 * @date 2025-06-10
 *
 * @copyright Copyright (c) 2025 Liu Yuanlin Personal.
 *
 */
#ifndef __HDL_CRC_H__
#define __HDL_CRC_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化硬件CRC外设
 * 
 * @return int 0: 成功, -1: 失败
 */
int HDL_CRC_Init(void);

/**
 * @brief 反初始化硬件CRC外设
 */
void HDL_CRC_DeInit(void);

/**
 * @brief 使用硬件CRC计算Modbus CRC16
 * 
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值（已进行字节交换，可直接用于发送）
 */
uint16_t HDL_CRC16_Modbus(const uint8_t *_pBuf, uint16_t _usLen);

/**
 * @brief 使用硬件CRC计算Modbus CRC16（带初始值）
 * 
 * @param crc 初始CRC值
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值（已进行字节交换，可直接用于发送）
 */
uint16_t HDL_CRC16_Modbus_With(uint16_t crc, const uint8_t *_pBuf, uint16_t _usLen);

/**
 * @brief 使用硬件CRC计算CCITT-FALSE CRC16
 * 
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值
 */
uint16_t HDL_CRC16_CCITT_FALSE(const uint8_t *_pBuf, uint16_t _usLen);

/**
 * @brief 使用硬件CRC计算CCITT-FALSE CRC16（带初始值）
 * 
 * @param crc 初始CRC值
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值
 */
uint16_t HDL_CRC16_CCITT_FALSE_With(uint16_t crc, const uint8_t *_pBuf, uint16_t _usLen);

#ifdef __cplusplus
}
#endif

#endif /* __HDL_CRC_H__ */