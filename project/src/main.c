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
#include "pid_q32.h"
#include <string.h>
#include <stdlib.h>
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

// 预先计算的电压转换因子（Q15定点数）
#define VREF                     (3.3f)                               // 根据实际电压修改
#define ADC_MAX_VALUE            (4095)                               // ADC最大值
#define VOLTAGE_SCALE_FACTOR_Q15 (uint32_t)((VREF / 4096.0f) * 32768) // Q15格式

#define ADC_RANK_NUM             6                     // ADC采样通道数
volatile uint16_t adc_buffer[ADC_RANK_NUM]      = {0}; // ADC采样数据缓冲区
volatile uint16_t adc_buffer_init[ADC_RANK_NUM] = {0}; // ADC采样数据缓冲区
volatile uint16_t filtered_adc[ADC_RANK_NUM]    = {0};

#define DMA1_CHANNEL1_MEMORY_BASE_ADDR ((uint32_t)adc_buffer) // DMA1通道1内存地址
#define DMA1_CHANNEL1_BUFFER_SIZE      (ADC_RANK_NUM)         // DMA1通道1缓冲区大小,单位是传输个数

#define ADC_VIN_RANK_IDX               0 // LLC输入电压,PA1
#define ADC_IO_RANK_IDX                1 // Buck输出电流,PA2
#define ADC_VO_TOTAL_RANK_IDX          2 // Buck输出电压,PA3
#define ADC_VO_MID_RANK_IDX            3 // Buck输出电压 MID,PA6
#define ADC_IIN_RANK_IDX               4 // LLC输入电流,PA7
#define ADC_V_LLC_RANK_IDX             5 // LLC输出电压,PB2

#define SCALE_ADC_VALUE_TO_INPUT_VOLT  (1.0f * (VREF / ADC_MAX_VALUE))
#define SCALE_ADC_VALUE_TO_BUCK_CURR   ((1000.0f / 185.0f) * (VREF / ADC_MAX_VALUE))
#define SCALE_ADC_VALUE_TO_BUCK_VOLT   (111.17f * (VREF / ADC_MAX_VALUE))
#define SCALE_ADC_VALUE_TO_VO_MID_VOLT (1.0f * (VREF / ADC_MAX_VALUE))
#define SCALE_ADC_VALUE_TO_LLC_CURR    ((1000.0f / 185.0f) * (VREF / ADC_MAX_VALUE))
#define SCALE_ADC_VALUE_TO_LLC_VOLT    (109.65f * (VREF / ADC_MAX_VALUE))

#define SCALE_INPUT_VOLT_TO_ADC_VALUE  (1.0f / SCALE_ADC_VALUE_TO_INPUT_VOLT)
#define SCALE_BUCK_CURR_TO_ADC_VALUE   (1.0f / SCALE_ADC_VALUE_TO_BUCK_CURR)
#define SCALE_BUCK_VOLT_TO_ADC_VALUE   (1.0f / SCALE_ADC_VALUE_TO_BUCK_VOLT)
#define SCALE_VO_MID_VOLT_TO_ADC_VALUE (1.0f / SCALE_ADC_VALUE_TO_VO_MID_VOLT)
#define SCALE_LLC_CURR_TO_ADC_VALUE    (1.0f / SCALE_ADC_VALUE_TO_LLC_CURR)
#define SCALE_LLC_VOLT_TO_ADC_VALUE    (1.0f / SCALE_ADC_VALUE_TO_LLC_VOLT)

float votlage_debug[ADC_RANK_NUM]       = {0}; // 电压调试数据
float adc_to_target_scale[ADC_RANK_NUM] = {
    SCALE_ADC_VALUE_TO_INPUT_VOLT,
    SCALE_ADC_VALUE_TO_BUCK_CURR,
    SCALE_ADC_VALUE_TO_BUCK_VOLT,
    SCALE_ADC_VALUE_TO_VO_MID_VOLT,
    SCALE_ADC_VALUE_TO_LLC_CURR,
    SCALE_ADC_VALUE_TO_LLC_VOLT,
};

// 安全限制
// 输入输出范围
// PWM频率：LLC 谐振点160KHz，100-400KHz，输入限流5A。
#define TMR1_CLK_FREQ 120000000UL // TMR1时钟频率为120MHz
// LLC PWM频率上限
#define LLC_FREQUENCY_UPPER_LIMIT 400000UL
// LLC PWM频率下限
#define LLC_FREQUENCY_LOWER_LIMIT 100000UL
// LLC PWM周期寄存器上限
#define LLC_PWM_PERIOD_UPPER_LIMIT ((TMR1_CLK_FREQ / LLC_FREQUENCY_LOWER_LIMIT) - 1) // 1200-1
// LLC PWM周期寄存器下限
#define LLC_PWM_PERIOD_LOWER_LIMIT ((TMR1_CLK_FREQ / LLC_FREQUENCY_UPPER_LIMIT) - 1) // 300-1
// LLC 输入电流安全上限
#define LLC_INPUT_CURRENT_UPPER_LIMIT 5.0f // 5A
// LLC 输入电流下限
#define LLC_INPUT_CURRENT_LOWER_LIMIT 0.0f // 0A
// LLC 输入电流限制
#define LLC_INPUT_CURRENT_OC_LIMIT (4.0f) // 4A
// LLC输出过压保护
#define LLC_OV_ADC_VALUE(volt) ((volt) * SCALE_LLC_VOLT_TO_ADC_VALUE)
uint32_t LLC_OV_THRESHOLD = LLC_OV_ADC_VALUE(350);
// Buck输出过压保护
#define BUCK_OV_ADC_VALUE(volt) ((volt) * SCALE_BUCK_VOLT_TO_ADC_VALUE)
uint32_t BUCK_OV_THRESHOLD = BUCK_OV_ADC_VALUE(350);
// LLC输入过流保护阈值，由于是霍尔元件，需要稍后初始化
uint32_t LLC_OC_THRESHOLD = 2.5f * SCALE_LLC_CURR_TO_ADC_VALUE;

// PID控制器实例
Inc_PID_Q32_t llc_volt_pid;
Inc_PID_Q32_t llc_curr_freq_pid;
Inc_PID_Q32_t llc_curr_duty_cycle_pid;
// PID控制器的缩放因子
#define PID_SHIFT 12 

void inline llc_set_tmr_period(uint32_t period)
{
    tmr_period_value_set(TMR1, period);
    // 设置占空比为50%
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, period >> 1);
}

/**
 * @brief 设置LLC驱动PWM的频率，默认%50占空比。
 *
 * @param frequency
 */
void llc_set_pwm_frequency(uint32_t frequency)
{
    if (frequency > LLC_FREQUENCY_UPPER_LIMIT) {
        frequency = LLC_FREQUENCY_UPPER_LIMIT;
    }
    if (frequency < LLC_FREQUENCY_LOWER_LIMIT) {
        frequency = LLC_FREQUENCY_LOWER_LIMIT;
    }
    // 设置LLC PWM频率
    // 计算计数值
    uint32_t count = (TMR1_CLK_FREQ / frequency) - 1;
    llc_set_tmr_period(count);
}

void llc_set_pwm_duty_cycle(float duty_cycle)
{
    if (duty_cycle < 0.0f) {
        duty_cycle = 0.0f;
    }
    if (duty_cycle > 1.0f) {
        duty_cycle = 1.0f;
    }
    // 设置LLC PWM占空比
    uint32_t period = tmr_channel_value_get(TMR1, TMR_SELECT_CHANNEL_2); // 获取当前周期值
    uint32_t value  = (uint32_t)(period * duty_cycle);
    tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, value);
}

void buck_set_tmr_channel_value(uint32_t value)
{
    // 设置Buck TMR通道2的值
    tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_2, value);
}

uint32_t llc_curr_to_adc_value(float curr)
{
    // 将电流转换为ADC值
    return (uint32_t)(curr * SCALE_LLC_CURR_TO_ADC_VALUE + adc_buffer_init[ADC_IIN_RANK_IDX]);
}

void set_llc_volt_target_to_adc_value_q32(float target_llc_volt)
{
    uint32_t value       = target_llc_volt * SCALE_LLC_VOLT_TO_ADC_VALUE;
    llc_volt_pid.iTarget = value;
}

void user_pid_init()
{
    // Init all fields as zero.
    Inc_PID_Q32_Init(&llc_volt_pid);
    Inc_PID_Q32_Init(&llc_curr_freq_pid);
    Inc_PID_Q32_Init(&llc_curr_duty_cycle_pid);

    uint32_t llc_curr_oc_limit_adc_value = llc_curr_to_adc_value(LLC_INPUT_CURRENT_OC_LIMIT);
    llc_volt_pid.iFmax                   = llc_curr_oc_limit_adc_value << PID_SHIFT; // 放大
    llc_volt_pid.iFmin                   = 0;
    llc_volt_pid.iF                      = llc_volt_pid.iFmin;
    llc_volt_pid.P                       = 200 * 3;
    llc_volt_pid.I                       = 50 * 3;
    llc_volt_pid.D                       = 0;

    llc_curr_freq_pid.iFmax = LLC_PWM_PERIOD_UPPER_LIMIT << PID_SHIFT; // 放大
    llc_curr_freq_pid.iFmin = LLC_PWM_PERIOD_LOWER_LIMIT << PID_SHIFT; // 放大
    llc_curr_freq_pid.iF    = llc_curr_freq_pid.iFmin;                 // 初始为最大频率
    llc_curr_freq_pid.P     = 200 * 3;
    llc_curr_freq_pid.I     = 50 * 3;
    llc_curr_freq_pid.D     = 0;

    llc_curr_duty_cycle_pid.iFmax = ((LLC_PWM_PERIOD_LOWER_LIMIT + 1) >> 1) << PID_SHIFT; // 放大
    llc_curr_duty_cycle_pid.iFmin = 0;
    llc_curr_duty_cycle_pid.iF    = llc_curr_duty_cycle_pid.iFmax * 0.0f; // 初始占空比
    llc_curr_duty_cycle_pid.P     = 200 * 3;
    llc_curr_duty_cycle_pid.I     = 50 * 3;
    llc_curr_duty_cycle_pid.D     = 0;
}

void llc_output_enable()
{
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
}

void llc_output_disable()
{
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, FALSE);
    tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, FALSE);
}

void buck_output_enable()
{
    tmr_channel_enable(TMR15, TMR_SELECT_CHANNEL_2, TRUE);
}

void buck_output_disable()
{
    tmr_channel_enable(TMR15, TMR_SELECT_CHANNEL_2, FALSE);
}

void disable_all_output()
{
    llc_output_disable();
    buck_output_disable();
}

void enable_all_output()
{
    llc_output_enable();
    buck_output_enable();
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

int stage             = 0;
int protect_type      = 0;      // 0:无保护，1:输入过流保护，2:输出过压保护
float llc_volt_target        = 200.0f; // LLC目标电压，单位V
bool llc_volt_target_changed = false;  // LLC目标电压是否改变
int stage_debug = 0;
uint32_t interrupt_cnt = 0;
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
    // 等待ADC电源稳定，避免校准误差
    wk_delay_ms(200);
    wk_adc1_init();

    /* init tmr1 function. */
    wk_tmr1_init();

    /* init tmr15 function. */
    wk_tmr15_init();

    /* add user code begin 2 */
    scope_init();
    // ulog_init_user();
    // ULOG_INFO("AT32F421 WK Demo Start");

    // 关闭所有PWM输出，避免暂态
    disable_all_output();
    // 启动定时器
    tmr_counter_enable(TMR15, TRUE);
    tmr_counter_enable(TMR1, TRUE);
    // ADC触发源使能
    tmr_channel_enable(TMR15, TMR_SELECT_CHANNEL_1, TRUE);
    // 同样的Buck置完再频率后再使能驱动PWM输出，消除暂态
    tmr_output_enable(TMR15, TRUE); // 要使能output，不然无法触发ADC采集。
    // 延时一段时间获取ADC初始值，用于校准霍尔电流传感器
    wk_delay_ms(200);
    // 保存初始值
    memcpy((void *)adc_buffer_init, (void *)adc_buffer, sizeof(adc_buffer_init));
    // 初始化PID控制器
    user_pid_init();
    // 设置过流保护阈值
    LLC_OC_THRESHOLD = llc_curr_to_adc_value(LLC_INPUT_CURRENT_UPPER_LIMIT);

    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_HDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);

    // 初始LLC频率为最高频率
    llc_set_pwm_frequency(LLC_FREQUENCY_UPPER_LIMIT);
    // 初始LLC占空比
    llc_set_pwm_duty_cycle(0.0f);
    // 增大值Buck占空比增大输出增大，关闭设置为0
    buck_set_tmr_channel_value(0);
    // 设置完LLC的频率后再使能LLC的驱动PWM输出，消除暂态
    tmr_output_enable(TMR1, TRUE);
    // 启动LLC输出
    llc_output_enable();
    // 设置LLC目标电压
    set_llc_volt_target_to_adc_value_q32(llc_volt_target);
    // 设置阶段为1，表示初始化完成
    stage = 1;
    /* add user code end 2 */

    while (1) {
        /* add user code begin 3 */
        // 发送数据到JScope,12字节
        // SEGGER_RTT_Write(1, &rtt_data, sizeof(rtt_data));
        wk_delay_ms(1);
        if (llc_volt_target_changed) {
            // 如果LLC目标电压改变，更新PID目标
            set_llc_volt_target_to_adc_value_q32(llc_volt_target);
            llc_volt_target_changed = false; // 重置标志位
        }
        /* add user code end 3 */
    }
}

/* add user code begin 4 */

void adc_dma_handler()
{
    interrupt_cnt++;

#define FILTER_SHIFT 3 // 相当于除以8的滤波系数
    for (int i = 0; i < ADC_RANK_NUM; i++) {
        // 定点数一阶滤波: y[n] = (x[n] + 7*y[n-1]) / 8
        filtered_adc[i] = (adc_buffer[i] + ((uint32_t)filtered_adc[i] << FILTER_SHIFT) - filtered_adc[i]) >> FILTER_SHIFT;
    }

    // 将ADC值转换为电压、电流值
    votlage_debug[ADC_VIN_RANK_IDX]      = filtered_adc[ADC_VIN_RANK_IDX] * adc_to_target_scale[ADC_VIN_RANK_IDX];
    votlage_debug[ADC_IO_RANK_IDX]       = (filtered_adc[ADC_IO_RANK_IDX] - adc_buffer_init[ADC_IO_RANK_IDX]) * adc_to_target_scale[ADC_IO_RANK_IDX];
    votlage_debug[ADC_VO_TOTAL_RANK_IDX] = filtered_adc[ADC_VO_TOTAL_RANK_IDX] * adc_to_target_scale[ADC_VO_TOTAL_RANK_IDX];
    votlage_debug[ADC_VO_MID_RANK_IDX]   = filtered_adc[ADC_VO_MID_RANK_IDX] * adc_to_target_scale[ADC_VO_MID_RANK_IDX];
    votlage_debug[ADC_IIN_RANK_IDX]      = (filtered_adc[ADC_IIN_RANK_IDX] - adc_buffer_init[ADC_IIN_RANK_IDX]) * adc_to_target_scale[ADC_IIN_RANK_IDX];
    votlage_debug[ADC_V_LLC_RANK_IDX]    = filtered_adc[ADC_V_LLC_RANK_IDX] * adc_to_target_scale[ADC_V_LLC_RANK_IDX];
    // 定点数计算：adc_value * scale_factor >> 15
    // votlage_debug[i] = (adc_buffer[i] * VOLTAGE_SCALE_FACTOR_Q15 + 0x4000) >> 15;

    if ((adc_buffer[ADC_V_LLC_RANK_IDX] > LLC_OV_THRESHOLD) ||
        (adc_buffer[ADC_VO_TOTAL_RANK_IDX] > BUCK_OV_THRESHOLD)) {
        // LLC输出过压或Buck过压，禁用所有输出
        disable_all_output();
        protect_type = 2; // 设置保护类型为输出过压保护
    }

    if (adc_buffer[ADC_IIN_RANK_IDX] > LLC_OC_THRESHOLD) {
        // LLC输入过流保护
        disable_all_output();
        protect_type = 1; // 设置保护类型为输入过流保护
    }

    // 更新PID采样值
    // 缩放见@user_pid_init
    llc_volt_pid.iSampling            = filtered_adc[ADC_V_LLC_RANK_IDX];
    llc_curr_freq_pid.iSampling       = filtered_adc[ADC_IIN_RANK_IDX];
    llc_curr_duty_cycle_pid.iSampling = filtered_adc[ADC_IIN_RANK_IDX];
    static int result                        = 0;
    static uint32_t tmr_channel_value = 0;
    switch (stage) {
        case 0:
            // 初始化阶段
            break;
        case 1:
            // 使能输出，但是为最低电流
            llc_output_enable();
            stage = 2;
            break;
        case 2:
            // 正常运行阶段，占空比PID控制
            // 当前电压低于目标电压则会增大目标电流
            Inc_PID_Q32_Update_AddDelta(&llc_volt_pid);
            // 将电压PID的输出目标电流的对应ADC值作为频率PID的目标
            llc_curr_duty_cycle_pid.iTarget = llc_volt_pid.iF >> PID_SHIFT;
            // 当前电流低于目标电流则会增大通道寄存器值从而增大duty cycle，从而提高电流
            result = Inc_PID_Q32_Update_AddDelta(&llc_curr_duty_cycle_pid);
            // 将占空比PID的输出目标占空比对应的通道寄存器值作为LLC PWM定时器的通道寄存器值
            tmr_channel_value = llc_curr_duty_cycle_pid.iF >> PID_SHIFT;
            if (interrupt_cnt & 0x01)
            {
                // 奇数次中断，尝试将第一位小数四舍五入
                #if PID_SHIFT > 0
                tmr_channel_value += ((llc_curr_duty_cycle_pid.iF & (1UL << (PID_SHIFT - 1))) ? 1 : 0);
                #endif
            }
            tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, tmr_channel_value);

            if (result == 1) {
                // 如果占空比PID的输出到达上限无法在增大输出占空比切换到频率PID控制
                // stage = 3;
                stage_debug = 3;
            }
            break;
        case 3:
            // 正常运行阶段，频率PID控制
            // 当前电压低于目标电压则会增大目标电流
            Inc_PID_Q32_Update_AddDelta(&llc_volt_pid);
            // 将电压PID的输出目标电流的对应ADC值作为频率PID的目标
            llc_curr_freq_pid.iTarget = llc_volt_pid.iF >> PID_SHIFT;
            // 当前电流低于目标电流则会增大PERIOD寄存器值从而降低频率，使得频率靠近谐振点从而提高电流
            result = Inc_PID_Q32_Update_AddDelta(&llc_curr_freq_pid);
            // 将频率PID的输出目标频率的对应PERIOD寄存器值作为LLC PWM定时器的PERIOD寄存器值，默认为50%占空比
            llc_set_tmr_period(llc_curr_freq_pid.iF >> PID_SHIFT);
            if (result == -1) {
                // 如果频率PID输出到达下限无法在增大输出频率则切换到占空比PID控制
                stage = 2;
            }
            break;
        default:
            break;
    }
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
        adc_dma_handler();
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
/* add user code end 4 */
