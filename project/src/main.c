/* add user code begin Header */
/**
 **************************************************************************
 * @file     main.c
 * @brief    main program
 **************************************************************************
 *                       Copyright notice & Disclaimer
 *
 * The software Board Support Package (BSP) that is made available to
 * download from Artery official website is the copyrighted work of Artery.
 * Artery authorizes customers to use, copy, and distribute the BSP
 * software and its related documentation for the purpose of design and
 * development in conjunction with Artery microcontrollers. Use of the
 * software is governed by this copyright notice and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
 * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
 * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
 * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
 * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
 *
 **************************************************************************
 */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "at32f421_wk_config.h"
#include "wk_adc.h"
#include "wk_crc.h"
#include "wk_pwc.h"
#include "wk_tmr.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include <stdint.h>
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */
#define ADC_RANK_NUM 6                                        // ADC采样通道数
volatile uint16_t adc_buffer[ADC_RANK_NUM] = {0};             // ADC采样数据缓冲区
#define DMA1_CHANNEL1_MEMORY_BASE_ADDR ((uint32_t)adc_buffer) // DMA1通道1内存地址
#define DMA1_CHANNEL1_BUFFER_SIZE      (ADC_RANK_NUM)         // DMA1通道1缓冲区大小,单位是传输个数
float votlage_debug[ADC_RANK_NUM] = {0};                      // 电压调试数据

#define ADC_VIN_RANK_IDX      0 // LLC输入电压,PA1
#define ADC_IO_RANK_IDX       1 // Buck输出电流,PA2
#define ADC_VO_TOTAL_RANK_IDX 2 // Buck输出电压,PA3
#define ADC_VO_MID_RANK_IDX   3 // PA6
#define ADC_IIN_RANK_IDX      4 // LLC输入电流,PA7
#define ADC_V_LLC_RANK_IDX    5 // LLC输出电压,PB2

// 预先计算的电压转换因子（Q15定点数）
#define VREF                     (3.3f)                               // 根据实际电压修改
#define VOLTAGE_SCALE_FACTOR_Q15 (uint32_t)((VREF / 4096.0f) * 32768) // Q15格式
// 安全限制
// 输入输出范围
// PWM频率：LLC 谐振点160KHz，100-300KHz，Buck 100KHz
#define LLC_PWM_PERIOD_UPPER_LIMIT  1200  // LLC PWM周期上限
#define LLC_PWM_PERIOD_LOWER_LIMIT  400   // LLC PWM周期下限
#define BUCK_PWM_PERIOD_UPPER_LIMIT 1200  // Buck PWM周期上限
#define BUCK_PWM_PERIOD_LOWER_LIMIT 0     // Buck PWM周期下限
#define LLC_VOLTAGE_UPPER_LIMIT     (200) // LLC输出电压上限,单位是V

#include "pid_q32.h"
// PID控制器实例
Inc_PID_Q32_t llc_volt_pid;
Inc_PID_Q32_t llc_curr_pid;
Inc_PID_Q32_t buck_volt_pid;

void llc_set_tmr_period(uint32_t period)
{
    tmr_period_value_set(TMR1, period);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, period >> 1); // 设置占空比为50%
}

void llc_set_pwm_frequency(uint32_t frequency)
{
    // 设置LLC PWM频率
    // 计算计数值
#define TMR1_CLK_FREQ 120000000 // TMR1时钟频率为120MHz
    uint32_t count = (TMR1_CLK_FREQ / frequency) - 1;
    llc_set_tmr_period(count);
}

void buck_set_tmr_channel_value(uint32_t value)
{
    // 设置Buck TMR通道2的值
    tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_2, value);
}

#define PID_SHIFT 12 // PID控制器的缩放因子
void user_pid_init()
{
    // Init all fields as zero.
    Inc_PID_Q32_Init(&llc_volt_pid);
    Inc_PID_Q32_Init(&llc_curr_pid);
    Inc_PID_Q32_Init(&buck_volt_pid);

    llc_volt_pid.iFmax = LLC_PWM_PERIOD_UPPER_LIMIT << PID_SHIFT; // 放大
    llc_volt_pid.iFmin = LLC_PWM_PERIOD_LOWER_LIMIT << PID_SHIFT; // 放大
    llc_volt_pid.iF    = llc_volt_pid.iFmin;
    llc_volt_pid.P     = 108 * 7;
    llc_volt_pid.I     = 78 * 17;
    llc_volt_pid.D     = 0;

    llc_curr_pid.iFmax = LLC_PWM_PERIOD_UPPER_LIMIT << PID_SHIFT; // 放大
    llc_curr_pid.iFmin = LLC_PWM_PERIOD_LOWER_LIMIT << PID_SHIFT; // 放大
    llc_curr_pid.iF    = llc_curr_pid.iFmin;
    llc_curr_pid.P     = 108 * 7;
    llc_curr_pid.I     = 78 * 17;
    llc_curr_pid.D     = 0;

    buck_volt_pid.iFmax = BUCK_PWM_PERIOD_UPPER_LIMIT << PID_SHIFT; // 放大
    buck_volt_pid.iFmin = BUCK_PWM_PERIOD_LOWER_LIMIT << PID_SHIFT; // 放大
    buck_volt_pid.iF    = buck_volt_pid.iFmin;
    buck_volt_pid.P     = 108 * 7;
    buck_volt_pid.I     = 78 * 17;
    buck_volt_pid.D     = 0;
}

#include "log.h"
uint8_t buf[2048]; // 定义全局变量

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

/* add user code end 0 */

/**
 * @brief main function.
 * @param  none
 * @retval none
 */
int main(void)
{
    /* add user code begin 1 */

    /* add user code end 1 */

    /* system clock config. */
    wk_system_clock_config();

    /* config periph clock. */
    wk_periph_clock_config();

    /* init pwc function. */
    wk_pwc_init();

    /* nvic config. */
    wk_nvic_config();

    /* timebase config. */
    wk_timebase_init();

    /* init gpio function. */
    wk_gpio_config();

    /* init dma1 channel1 */
    wk_dma1_channel1_init();
    /* config dma channel transfer parameter */
    /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
    wk_dma_channel_config(DMA1_CHANNEL1,
                          (uint32_t)&ADC1->odt,
                          DMA1_CHANNEL1_MEMORY_BASE_ADDR,
                          DMA1_CHANNEL1_BUFFER_SIZE);
    dma_channel_enable(DMA1_CHANNEL1, TRUE);

    /* init crc function. */
    wk_crc_init();

    /* init adc1 function. */
    wk_adc1_init();

    /* init tmr1 function. */
    wk_tmr1_init();

    /* init tmr15 function. */
    wk_tmr15_init();

    /* add user code begin 2 */
    user_pid_init();
    scope_init();
    // ulog_init_user();
    // ULOG_INFO("AT32F421 WK Demo Start");

    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_HDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);

    llc_set_pwm_frequency(160000U);
    tmr_channel_enable(TMR15, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_counter_enable(TMR15, TRUE);

    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
    tmr_counter_enable(TMR1, TRUE);
        uint16_t rtt_data[6]={0};

    /* add user code end 2 */

    while (1) {
        /* add user code begin 3 */
        wk_delay_ms(100); 
        rtt_data[0] = votlage_debug[ADC_VIN_RANK_IDX] * 1000;      // 输入电压
        rtt_data[1] = votlage_debug[ADC_IO_RANK_IDX] * 1000;       // Buck输出电流
        rtt_data[2] = votlage_debug[ADC_VO_TOTAL_RANK_IDX] * 1000; // Buck输出电压
        rtt_data[3] = votlage_debug[ADC_VO_MID_RANK_IDX] * 1000;   // Buck中点电压
        rtt_data[4] = votlage_debug[ADC_IIN_RANK_IDX] * 1000;      // LLC输入电流
        rtt_data[5] = votlage_debug[ADC_V_LLC_RANK_IDX] * 1000;    // LLC输出电压
        // 发送数据到JScope,12字节
        SEGGER_RTT_Write(1, &rtt_data, sizeof(rtt_data));
        /* add user code end 3 */
    }
}

/* add user code begin 4 */
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

#include <stdlib.h>

void calculate_divmod(int dividend, int divisor, int *quot, int *rem)
{
    div_t result = div(dividend, divisor);
    *quot        = result.quot;
    *rem         = result.rem;
}

/**
 * @brief  this function handles DMA1 Channel 1 handler.
 * @param  none
 * @retval none
 */
void DMA1_Channel1_IRQHandler(void)
{
    /* add user code begin DMA1_Channel1_IRQ 0 */
    /* check if the DMA1 Channel 1 transfer complete interrupt flag is set */
    if (dma_interrupt_flag_get(DMA1_FDT1_FLAG) != RESET) {
        gpio_bits_set(IO1_GPIO_PORT, IO1_PIN);
        /* clear the DMA1 Channel 1 transfer complete interrupt flag */
        for (int i = 0; i < ADC_RANK_NUM; i++) {
            votlage_debug[i] = adc_buffer[i] * 3.3f / 4096.0f; // 将ADC值转换为电压值
            // 定点数计算：adc_value * scale_factor >> 15
            // votlage_debug[i] = (adc_buffer[i] * VOLTAGE_SCALE_FACTOR_Q15 + 0x4000) >> 15;
        }
        dma_flag_clear(DMA1_FDT1_FLAG);
        /* add user code here to handle the transfer complete event */
        gpio_bits_reset(IO1_GPIO_PORT, IO1_PIN);
    }

    /* check if the DMA1 Channel 1 half transfer interrupt flag is set */
    if (dma_interrupt_flag_get(DMA1_HDT1_FLAG) != RESET) {
        /* clear the DMA1 Channel 1 half transfer interrupt flag */
        dma_flag_clear(DMA1_HDT1_FLAG);
        /* add user code here to handle the half transfer event */
    }

    /* check if the DMA1 Channel 1 transfer error interrupt flag is set */
    if (dma_interrupt_flag_get(DMA1_DTERR1_FLAG) != RESET) {
        /* clear the DMA1 Channel 1 transfer error interrupt flag */
        dma_flag_clear(DMA1_DTERR1_FLAG);
        /* add user code here to handle the transfer error event */
    }
    /* add user code end DMA1_Channel1_IRQ 0 */
    /* add user code begin DMA1_Channel1_IRQ 1 */

    /* add user code end DMA1_Channel1_IRQ 1 */
}

/* add user code end 4 */
