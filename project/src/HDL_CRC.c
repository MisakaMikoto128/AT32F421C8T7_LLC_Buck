/**
 * @file HDL_CRC.c
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief AT32硬件CRC外设实现CRC16算法接口
 * @version 0.1
 * @date 2025-06-10
 *
 * @copyright Copyright (c) 2025 Liu Yuanlin Personal.
 *
 */
#include "HDL_CRC.h"
#include <stddef.h>
#include "at32f421_crc.h"
#include "at32f421_crm.h"

/**
 * @brief 初始化硬件CRC外设
 * 
 * @return int 0: 成功, -1: 失败
 */
int HDL_CRC_Init(void)
{
    // 使能CRC时钟
    crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, TRUE);
    
    // 复位CRC外设
    crc_data_reset();
    
    return 0;
}

/**
 * @brief 反初始化硬件CRC外设
 */
void HDL_CRC_DeInit(void)
{
    // 禁用CRC时钟
    crm_periph_clock_enable(CRM_CRC_PERIPH_CLOCK, FALSE);
}

/**
 * @brief 配置CRC外设为Modbus模式
 * 
 * 多项式: 0x8005 (反转的0xA001)
 * 初始值: 0xFFFF
 * 输入反转: 按字节反转
 * 输出反转: 反转
 */
static void HDL_CRC_Config_Modbus(void)
{
    crc_init_data_set(0xFFFF);
    crc_poly_size_set(CRC_POLY_SIZE_16B);
    crc_poly_value_set(0x8005);  // Modbus多项式
    crc_reverse_input_data_set(CRC_REVERSE_INPUT_BY_BYTE);
    crc_reverse_output_data_set(CRC_REVERSE_OUTPUT_DATA);
}

/**
 * @brief 配置CRC外设为CCITT-FALSE模式
 * 
 * 多项式: 0x1021
 * 初始值: 0xFFFF
 * 输入反转: 不反转
 * 输出反转: 不反转
 */
static void HDL_CRC_Config_CCITT_FALSE(void)
{
    crc_init_data_set(0xFFFF);
    crc_poly_size_set(CRC_POLY_SIZE_16B);
    crc_poly_value_set(0x1021);  // CCITT-FALSE多项式
    crc_reverse_input_data_set(CRC_REVERSE_INPUT_NO_AFFECTE);
    crc_reverse_output_data_set(CRC_REVERSE_OUTPUT_NO_AFFECTE);
}

/**
 * @brief 使用硬件CRC计算Modbus CRC16
 * 
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值（已进行字节交换，可直接用于发送）
 */
uint16_t HDL_CRC16_Modbus(const uint8_t *_pBuf, uint16_t _usLen)
{
    uint32_t crc_result;
    uint16_t i;
    
    if (_pBuf == NULL || _usLen == 0)
        return 0;
    
    // 配置CRC为Modbus模式
    HDL_CRC_Config_Modbus();
    
    // 复位CRC数据寄存器
    crc_data_reset();
    
    // 逐字节计算CRC
    for (i = 0; i < _usLen; i++)
    {
        crc_one_word_calculate(_pBuf[i]);
    }
    
    // 获取CRC结果并截取低16位
    crc_result = crc_data_get();
    
    return (uint16_t)(crc_result & 0xFFFF);
}

/**
 * @brief 使用硬件CRC计算Modbus CRC16（带初始值）
 * 
 * @param crc 初始CRC值
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值（已进行字节交换，可直接用于发送）
 */
uint16_t HDL_CRC16_Modbus_With(uint16_t crc, const uint8_t *_pBuf, uint16_t _usLen)
{
    uint32_t crc_result;
    uint16_t i;
    
    if (_pBuf == NULL || _usLen == 0)
        return crc;
    
    // 配置CRC为Modbus模式
    HDL_CRC_Config_Modbus();
    
    // 设置初始值
    crc_init_data_set(crc);
    crc_data_reset();
    
    // 逐字节计算CRC
    for (i = 0; i < _usLen; i++)
    {
        crc_one_word_calculate(_pBuf[i]);
    }
    
    // 获取CRC结果并截取低16位
    crc_result = crc_data_get();
    
    return (uint16_t)(crc_result & 0xFFFF);
}

/**
 * @brief 使用硬件CRC计算CCITT-FALSE CRC16
 * 
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值
 */
uint16_t HDL_CRC16_CCITT_FALSE(const uint8_t *_pBuf, uint16_t _usLen)
{
    uint32_t crc_result;
    uint16_t i;
    uint32_t temp_data;
    
    if (_pBuf == NULL || _usLen == 0)
        return 0;
    
    // 配置CRC为CCITT-FALSE模式
    HDL_CRC_Config_CCITT_FALSE();
    
    // 复位CRC数据寄存器
    crc_data_reset();
    
    // 逐字节计算CRC
    // 对于CCITT-FALSE，需要将字节左移到高位
    for (i = 0; i < _usLen; i++)
    {
        temp_data = ((uint32_t)_pBuf[i]) << 24;  // 将字节放到最高位
        crc_one_word_calculate(temp_data);
    }
    
    // 获取CRC结果并截取高16位
    crc_result = crc_data_get();
    
    return (uint16_t)((crc_result >> 16) & 0xFFFF);
}

/**
 * @brief 使用硬件CRC计算CCITT-FALSE CRC16（带初始值）
 * 
 * @param crc 初始CRC值
 * @param _pBuf 数据缓冲区指针
 * @param _usLen 数据长度
 * @return uint16_t CRC16值
 */
uint16_t HDL_CRC16_CCITT_FALSE_With(uint16_t crc, const uint8_t *_pBuf, uint16_t _usLen)
{
    uint32_t crc_result;
    uint16_t i;
    uint32_t temp_data;
    
    if (_pBuf == NULL || _usLen == 0)
        return crc;
    
    // 配置CRC为CCITT-FALSE模式
    HDL_CRC_Config_CCITT_FALSE();
    
    // 设置初始值（需要左移到高16位）
    crc_init_data_set(((uint32_t)crc) << 16);
    crc_data_reset();
    
    // 逐字节计算CRC
    for (i = 0; i < _usLen; i++)
    {
        temp_data = ((uint32_t)_pBuf[i]) << 24;  // 将字节放到最高位
        crc_one_word_calculate(temp_data);
    }
    
    // 获取CRC结果并截取高16位
    crc_result = crc_data_get();
    
    return (uint16_t)((crc_result >> 16) & 0xFFFF);
}