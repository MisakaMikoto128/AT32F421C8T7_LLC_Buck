/**
 * @file crc.h
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2022-11-08
 *
 * @copyright Copyright (c) 2022 Liu Yuanlin Personal.
 *
 */
#ifndef CRC_H
#define CRC_H
#include <stdint.h>
// 多项式
#define CRC16_IBM_POLYNOMIAL    0x8005UL
#define CRC16_MODBUS_POLYNOMIAL 0x8005UL
#define CRC16_CCITT_POLYNOMIAL  0x1021UL
#define CRC16_XMODEM_POLYNOMIAL 0x1021UL
#define CRC16_USB_POLYNOMIAL    0x8005UL
#define CRC32_POLYNOMIAL        0x04C11DB7UL
// 初始值
#define CRC16_IBM_INIT    0x0000UL
#define CRC16_MODBUS_INIT 0xFFFFUL
#define CRC32_INIT        0xFFFFFFFFUL
// 结果异或值
#define CRC16_IBM_XOR    0x0000UL
#define CRC16_MODBUS_XOR 0x0000UL
#define CRC32_XOR        0xFFFFFFFFUL
// 是否反转输入数据
#define CRC16_IBM_REFIN    1UL
#define CRC16_MODBUS_REFIN 1UL
#define CRC32_REFIN        1UL
// 是否反转输出数据
#define CRC16_IBM_REFOUT    1UL
#define CRC16_MODBUS_REFOUT 1UL
#define CRC32_REFOUT        1UL

uint16_t CRC16_Modbus(const uint8_t *_pBuf, uint16_t _usLen);
#define CRC16_Modbus_Start() (CRC16_MODBUS_INIT)
uint16_t CRC16_Modbus_With(uint16_t crc, const uint8_t *_pBuf, uint16_t _usLen);

uint32_t CRC32(const uint8_t *_pBuf, uint32_t _ulLen);
#define CRC32_Start()     (CRC32_INIT)
#define CRC32_Get(_ulCRC) (_ulCRC ^ CRC32_XOR)
/**
 * @brief 使用_ulCRC作为CRC初始值计算CRC32。使用CRC32_Start()作为最开始的初始值。
 *
 * @param _pBuf
 * @param _ulLen
 * @param _ulCRC 当前CRC值
 * @return uint32_t 计算后的CRC值，但是没有取反，需要使用CRC32_Get()取反。
 */
uint32_t CRC32_With(const uint8_t *_pBuf, uint32_t _ulLen, uint32_t _ulCRC);
/**
 * @brief 计算CRC16 CCITT-FALSE校验值
 * 
 * 多项式: 0x1021 (x^16 + x^12 + x^5 + 1)
 * 初始值: 0xFFFF
 * 输入反转: 否
 * 输出反转: 否
 * 结果异或值: 0x0000
 * 
 * @param _pBuf 参与校验的数据缓冲区
 * @param _usLen 数据长度
 * @return uint16_t 计算得到的CRC16值
 */
uint16_t CRC16_CCITT_FALSE(const uint8_t *_pBuf, uint16_t _usLen);

/**
 * @brief 使用指定初始值计算CRC16 CCITT-FALSE校验值
 * 
 * @param crc 初始CRC值
 * @param _pBuf 参与校验的数据缓冲区
 * @param _usLen 数据长度
 * @return uint16_t 计算得到的CRC16值
 */
uint16_t CRC16_CCITT_FALSE_With(uint16_t crc, const uint8_t *_pBuf, uint16_t _usLen);
#endif // !CRC_H
