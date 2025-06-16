/**
 * @file communication.c
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2025-06-16
 * @last modified 2025-06-16
 *
 * @copyright Copyright (c) 2025 Liu Yuanlin Personal.
 *
 */
#include "communication.h"
#include "wk_usart.h"
#include <string.h>
#include <stdlib.h>
#include "log.h"
#include "crc.h"

#define COUNTOF(a)            (sizeof(a) / sizeof(*(a)))
#define USART2_TX_BUFFER_SIZE (8)
#define USART2_RX_BUFFER_SIZE (20)
uint8_t usart2_tx_buffer[USART2_TX_BUFFER_SIZE];
uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
volatile uint8_t usart2_tx_counter = 0x00;
volatile uint8_t usart2_rx_counter = 0x00;
uint8_t usart2_tx_buffer_size      = USART2_TX_BUFFER_SIZE;
uint8_t usart2_rx_buffer_size      = USART2_RX_BUFFER_SIZE;
bool usart2_rx_idle_flag           = false;

#define USART1_TX_BUFFER_SIZE (USART2_TX_BUFFER_SIZE)
#define USART1_RX_BUFFER_SIZE (USART2_RX_BUFFER_SIZE)
uint8_t usart1_tx_buffer[USART1_TX_BUFFER_SIZE];
uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
volatile uint8_t usart1_tx_counter = 0x00;
volatile uint8_t usart1_rx_counter = 0x00;
uint8_t usart1_tx_buffer_size      = USART1_TX_BUFFER_SIZE;
uint8_t usart1_rx_buffer_size      = USART1_RX_BUFFER_SIZE;
bool usart1_rx_idle_flag           = false;

void communication_init()
{
    // 串口相关：数据位个数9位(包含奇偶校验位)，奇校验，1位停止位，9600波特率
    usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);
    usart_interrupt_enable(USART1, USART_TDBE_INT, FALSE);
    usart_interrupt_enable(USART1, USART_ERR_INT, TRUE);
    usart_interrupt_enable(USART1, USART_PERR_INT, TRUE);
    usart_interrupt_enable(USART1, USART_IDLE_INT, TRUE);

    usart_interrupt_enable(USART2, USART_RDBF_INT, TRUE);
    usart_interrupt_enable(USART2, USART_TDBE_INT, FALSE);
    usart_interrupt_enable(USART2, USART_ERR_INT, TRUE);
    usart_interrupt_enable(USART2, USART_PERR_INT, TRUE);
    usart_interrupt_enable(USART2, USART_IDLE_INT, TRUE);
}

extern uint32_t power_wdg_cnt;
bool is_power_source_launched();
void llc_target_set(float value);
float get_llc_volt_from_adc_value();
float get_llc_input_curr_from_adc_value();
void power_source_launch();
void power_source_shutdown();

void communication_poll()
{
    // 处理串口数据
    do {
        if (!usart2_rx_idle_flag) {
            break;
        } else {
            usart2_rx_idle_flag = false;
        }

        int rx_bytes_num  = usart2_rx_counter;
        usart2_rx_counter = 0;

        if (rx_bytes_num < 4) {
            break;
        }

        uint16_t crc_res = CRC16_CCITT_FALSE(usart2_rx_buffer, rx_bytes_num);
        if (crc_res != 0) {
            break;
        }

        power_wdg_cnt = 0;
        uint8_t func  = usart2_rx_buffer[0];
        uint8_t value = usart2_rx_buffer[1];
        switch (func) {
            case 0x20:
                if (value > 220) {
                    value = 220;
                } else if (value < 10) {
                    value = 10;
                }
                llc_target_set(value);
                break;
            case 0x80:
                if (value == 0xAA) {
                    power_source_launch();
                } else if (
                    value == 0xBB) {
                    power_source_shutdown();
                }
                break;
            default:
                break;
        }

        float llc_volt             = get_llc_volt_from_adc_value();
        float llc_input_curr       = get_llc_input_curr_from_adc_value();
        int16_t llc_volt_u16       = llc_volt * 100;
        int16_t llc_input_curr_u16 = llc_input_curr * 100;
        int idx                    = 0;

        usart2_tx_buffer[idx++] = 0x80;
        usart2_tx_buffer[idx++] = is_power_source_launched() ? 0xAA : 0xBB;
        usart2_tx_buffer[idx++] = llc_volt_u16 >> 8;
        usart2_tx_buffer[idx++] = llc_volt_u16 & 0xFF;
        usart2_tx_buffer[idx++] = llc_input_curr_u16 >> 8;
        usart2_tx_buffer[idx++] = llc_input_curr_u16 & 0xFF;
        crc_res                 = CRC16_CCITT_FALSE(usart2_tx_buffer, idx);
        usart2_tx_buffer[idx++] = crc_res & 0xFF;
        usart2_tx_buffer[idx++] = crc_res >> 8;

        usart2_tx_counter = 0;
        usart_interrupt_enable(USART2, USART_TDBE_INT, TRUE);

        memcpy(usart1_tx_buffer, usart2_tx_buffer, sizeof(usart1_tx_buffer));
        usart1_tx_counter = 0;
        usart_interrupt_enable(USART1, USART_TDBE_INT, TRUE);
    } while (0);
}

/**
 * @brief  this function handles usart1 handler.
 * @param  none
 * @retval none
 */
void USART1_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART1, USART_RDBF_FLAG) != RESET) {
        if (usart1_rx_counter < usart1_rx_buffer_size) {
            /* read one byte from the receive data register */
            usart1_rx_buffer[usart1_rx_counter++] = usart_data_receive(USART1);
        } else {
            volatile uint8_t ch = usart_data_receive(USART1);
        }
        usart_flag_clear(USART1, USART_RDBF_FLAG);
    }

    if (usart_interrupt_flag_get(USART1, USART_TDBE_FLAG) != RESET) {
        /* write one byte to the transmit data register */
        usart_data_transmit(USART1, usart1_tx_buffer[usart1_tx_counter++]);

        if (usart1_tx_counter == usart1_tx_buffer_size) {
            /* disable the usart1 transmit interrupt */
            usart_interrupt_enable(USART1, USART_TDBE_INT, FALSE);
        }
    }

    /* 处理帧错误中断 */
    if (usart_interrupt_flag_get(USART1, USART_FERR_FLAG) != RESET) {
        /* 清除帧错误标志 */
        usart_flag_clear(USART1, USART_FERR_FLAG);
        /* 可以在这里添加帧错误处理代码 */
    }

    /* 处理噪声错误中断 */
    if (usart_interrupt_flag_get(USART1, USART_NERR_FLAG) != RESET) {
        /* 清除噪声错误标志 */
        usart_flag_clear(USART1, USART_NERR_FLAG);
        /* 可以在这里添加噪声错误处理代码 */
    }

    /* 处理奇偶校验错误中断 */
    if (usart_interrupt_flag_get(USART1, USART_PERR_FLAG) != RESET) {
        volatile uint8_t ch = usart_data_receive(USART1);
        /* 清除奇偶校验错误标志 */
        usart_flag_clear(USART1, USART_PERR_FLAG);
        /* 可以在这里添加奇偶校验错误处理代码 */
    }

    /* 处理接收器溢出错误中断 */
    if (usart_interrupt_flag_get(USART1, USART_ROERR_FLAG) != RESET) {
        /* 清除接收器溢出错误标志 */
        usart_flag_clear(USART1, USART_ROERR_FLAG);
        /* 可以在这里添加接收器溢出错误处理代码 */
    }

    /* 处理空闲帧中断 */
    if (usart_interrupt_flag_get(USART1, USART_IDLEF_FLAG) != RESET) {
        usart1_rx_idle_flag = true;
        /* 清除空闲帧标志 */
        usart_flag_clear(USART1, USART_IDLEF_FLAG);
        /* 可以在这里添加空闲帧处理代码 */
    }

    /* 处理发送完成中断 */
    if (usart_interrupt_flag_get(USART1, USART_TDC_FLAG) != RESET) {
        /* 清除发送完成标志 */
        usart_flag_clear(USART1, USART_TDC_FLAG);
        /* 可以在这里添加发送完成处理代码 */
    }

    /* 处理断帧中断 */
    if (usart_interrupt_flag_get(USART1, USART_BFF_FLAG) != RESET) {
        /* 清除断帧标志 */
        usart_flag_clear(USART1, USART_BFF_FLAG);
        /* 可以在这里添加断帧处理代码 */
    }

    /* 处理CTS变化中断 */
    if (usart_interrupt_flag_get(USART1, USART_CTSCF_FLAG) != RESET) {
        /* 清除CTS变化标志 */
        usart_flag_clear(USART1, USART_CTSCF_FLAG);
        /* 可以在这里添加CTS变化处理代码 */
    }
}
/**
 * @brief  this function handles usart2 handler.
 * @param  none
 * @retval none
 */
void USART2_IRQHandler(void)
{
    if (usart_interrupt_flag_get(USART2, USART_RDBF_FLAG) != RESET) {
        if (usart2_rx_counter < usart2_rx_buffer_size) {
            /* read one byte from the receive data register */
            usart2_rx_buffer[usart2_rx_counter++] = usart_data_receive(USART2);
        } else {
            volatile uint8_t ch = usart_data_receive(USART2);
        }
        usart_flag_clear(USART2, USART_RDBF_FLAG);
    }

    if (usart_interrupt_flag_get(USART2, USART_TDBE_FLAG) != RESET) {
        /* write one byte to the transmit data register */
        usart_data_transmit(USART2, usart2_tx_buffer[usart2_tx_counter++]);

        if (usart2_tx_counter == usart2_tx_buffer_size) {
            /* disable the usart2 transmit interrupt */
            usart_interrupt_enable(USART2, USART_TDBE_INT, FALSE);
        }
    }

    /* 处理帧错误中断 */
    if (usart_interrupt_flag_get(USART2, USART_FERR_FLAG) != RESET) {
        /* 清除帧错误标志 */
        usart_flag_clear(USART2, USART_FERR_FLAG);
        /* 可以在这里添加帧错误处理代码 */
    }

    /* 处理噪声错误中断 */
    if (usart_interrupt_flag_get(USART2, USART_NERR_FLAG) != RESET) {
        /* 清除噪声错误标志 */
        usart_flag_clear(USART2, USART_NERR_FLAG);
        /* 可以在这里添加噪声错误处理代码 */
    }

    /* 处理奇偶校验错误中断 */
    if (usart_interrupt_flag_get(USART2, USART_PERR_FLAG) != RESET) {
        volatile uint8_t ch = usart_data_receive(USART2);
        /* 清除奇偶校验错误标志 */
        usart_flag_clear(USART2, USART_PERR_FLAG);
        /* 可以在这里添加奇偶校验错误处理代码 */
    }

    /* 处理接收器溢出错误中断 */
    if (usart_interrupt_flag_get(USART2, USART_ROERR_FLAG) != RESET) {
        /* 清除接收器溢出错误标志 */
        usart_flag_clear(USART2, USART_ROERR_FLAG);
        /* 可以在这里添加接收器溢出错误处理代码 */
    }

    /* 处理空闲帧中断 */
    if (usart_interrupt_flag_get(USART2, USART_IDLEF_FLAG) != RESET) {
        usart2_rx_idle_flag = true;
        /* 清除空闲帧标志 */
        usart_flag_clear(USART2, USART_IDLEF_FLAG);
        /* 可以在这里添加空闲帧处理代码 */
    }

    /* 处理发送完成中断 */
    if (usart_interrupt_flag_get(USART2, USART_TDC_FLAG) != RESET) {
        /* 清除发送完成标志 */
        usart_flag_clear(USART2, USART_TDC_FLAG);
        /* 可以在这里添加发送完成处理代码 */
    }

    /* 处理断帧中断 */
    if (usart_interrupt_flag_get(USART2, USART_BFF_FLAG) != RESET) {
        /* 清除断帧标志 */
        usart_flag_clear(USART2, USART_BFF_FLAG);
        /* 可以在这里添加断帧处理代码 */
    }

    /* 处理CTS变化中断 */
    if (usart_interrupt_flag_get(USART2, USART_CTSCF_FLAG) != RESET) {
        /* 清除CTS变化标志 */
        usart_flag_clear(USART2, USART_CTSCF_FLAG);
        /* 可以在这里添加CTS变化处理代码 */
    }
}

// 内联汇编实现
static inline void udiv_mod(uint32_t dividend, uint32_t divisor, uint32_t *quotient, uint32_t *remainder)
{
    __asm volatile(
        "udiv %0, %2, %3\n\t"    // 商
        "mls %1, %0, %3, %2\n\t" // 余数 = dividend - quotient * divisor
        : "=r"(*quotient), "=r"(*remainder)
        : "r"(dividend), "r"(divisor)
        : "cc");
}

void calculate_divmod(int dividend, int divisor, int *quot, int *rem)
{
    div_t result = div(dividend, divisor);
    *quot        = result.quot;
    *rem         = result.rem;
}

static uint8_t buf[2048]; // 定义全局变量

void scope_init()
{
    SEGGER_RTT_ConfigUpBuffer(1, "JScope_u2u2u2u2u2u2", buf, 2048, SEGGER_RTT_MODE_NO_BLOCK_SKIP); // 初始化RTT模块

    /**
     * uint16_t rtt_data[8]={0};
     * ...
     * * // 发送数据到JScope,16字节
     * SEGGER_RTT_Write(1, &rtt_data, 16);
     */
}