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
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include <stdint.h>
#include "pid_q32.h"
#include <string.h>
#include <stdlib.h>
#include "crc.h"
#include "ccommon.h"
#include "wk_adc.h"
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
#define SCALE_ADC_VALUE_TO_LLC_CURR    ((1000.0f / 185.0f * 3.0f / 2.0f * 0.96f) * (VREF / ADC_MAX_VALUE))
#define SCALE_ADC_VALUE_TO_LLC_VOLT    (111.22f * (VREF / ADC_MAX_VALUE))

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
#define LLC_FREQUENCY_LOWER_LIMIT 200000UL // 116000UL
// LLC PWM周期寄存器上限
#define LLC_PWM_PERIOD_UPPER_LIMIT ((TMR1_CLK_FREQ / LLC_FREQUENCY_LOWER_LIMIT) - 1) // 1000-1
// LLC PWM周期寄存器下限
#define LLC_PWM_PERIOD_LOWER_LIMIT ((TMR1_CLK_FREQ / LLC_FREQUENCY_UPPER_LIMIT) - 1) // 300-1
// LLC Duty比较器映射
// (300-0)  -> (50%-0%) -> (150-0)
// LLC 输入电流安全上限
#define LLC_INPUT_CURRENT_UPPER_LIMIT 5.0f // 5A,TODO:这里限制要改，不然ADC测不到这么高。
// LLC 输入电流下限
#define LLC_INPUT_CURRENT_LOWER_LIMIT 0.0f // 0A
// LLC 输入电流限制
#define LLC_INPUT_CURRENT_OC_LIMIT (4.8f) // 4A，改成3.5A，因为ADC压根测不到4A。
// LLC输出过压保护
#define LLC_OV_ADC_VALUE(volt) ((volt) * SCALE_LLC_VOLT_TO_ADC_VALUE)
uint32_t LLC_OV_THRESHOLD = LLC_OV_ADC_VALUE(220);
// Buck输出过压保护
#define BUCK_OV_ADC_VALUE(volt) ((volt) * SCALE_BUCK_VOLT_TO_ADC_VALUE)
uint32_t BUCK_OV_THRESHOLD = BUCK_OV_ADC_VALUE(320);
// LLC输入过流保护阈值，由于是霍尔元件，需要稍后初始化
uint32_t LLC_OC_THRESHOLD       = 2.5f * SCALE_LLC_CURR_TO_ADC_VALUE;
uint32_t LLC_Devta_OC_THRESHOLD = LLC_INPUT_CURRENT_OC_LIMIT * SCALE_LLC_CURR_TO_ADC_VALUE;

uint32_t llc_curr_oc_limit_adc_value_small = 0;
uint32_t llc_curr_oc_limit_adc_value_upper = 0;

// PID控制器实例
Inc_PID_Q32_t llc_volt_pid;
Inc_PID_Q32_t llc_curr_freq_pid;
// PID控制器的缩放因子
#define PID_SHIFT    12
#define PID_SHIFT_14 14

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

void user_pid_init()
{
    // Init all fields as zero.
    Inc_PID_Q32_Init(&llc_volt_pid);
    Inc_PID_Q32_Init(&llc_curr_freq_pid);

    uint32_t llc_curr_oc_limit_adc_value    = llc_curr_to_adc_value(LLC_INPUT_CURRENT_OC_LIMIT);
    uint32_t llc_curr_lower_limit_adc_value = llc_curr_to_adc_value(-0.05f);
    uint32_t llc_curr_zero_limit_adc_value  = llc_curr_to_adc_value(0);
    llc_volt_pid.iFmax                      = llc_curr_oc_limit_adc_value << PID_SHIFT; // 放大
    llc_volt_pid.iFmin                      = llc_curr_lower_limit_adc_value << PID_SHIFT;
    llc_volt_pid.iF                         = llc_curr_zero_limit_adc_value << PID_SHIFT;
    llc_volt_pid.P                          = 20;
    llc_volt_pid.I                          = 200;
    llc_volt_pid.D                          = 10;

    llc_curr_freq_pid.iFmax = (LLC_PWM_PERIOD_UPPER_LIMIT + 1) << PID_SHIFT_14; // 1200放大
    llc_curr_freq_pid.iFmin = 20 << PID_SHIFT_14;                               // 放大
    llc_curr_freq_pid.iF    = llc_curr_freq_pid.iFmin;
    // 初始为最大频率最小占空比
    llc_curr_freq_pid.P = 50 * 1;
    llc_curr_freq_pid.I = 600 * 1;
    llc_curr_freq_pid.D = 1;
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

#include "log.h"

void scope_init();

int stage                    = 0;
int protect_type             = 0;     // 0:无保护，1:输入过流保护，2:输出过压保护
float llc_volt_target        = 10.0f; // LLC目标电压，单位V
bool llc_volt_target_changed = false; // LLC目标电压是否改变
int stage_debug              = 0;
uint32_t interrupt_cnt       = 0;
uint32_t oc_cnt              = 0;
uint32_t interrupt_pre_ticks = 0;

void set_llc_volt_target_to_adc_value_q32(float target_llc_volt)
{
    float setting_volt = 0.9908f * target_llc_volt + 3.6903f;
    if (setting_volt < 0) {
        setting_volt = 0;
    }
    uint32_t value       = setting_volt * SCALE_LLC_VOLT_TO_ADC_VALUE;
    llc_volt_pid.iTarget = value;
}

float get_llc_volt_from_adc_value()
{
    // 将ADC值转换为基础电压值
    float voltage = filtered_adc[ADC_V_LLC_RANK_IDX] * SCALE_ADC_VALUE_TO_LLC_VOLT;
    // 应用校准公式：实际电压 = (测量电压 - 3.6903) / 0.9908
    // 这个公式是@set_llc_volt_target_to_adc_value_q32中公式的反向转换
    voltage = (voltage - 3.6903f) / 0.9908f;
    if (voltage < 0) {
        voltage = 0;
    }
    return voltage;
}

float get_llc_input_curr_from_adc_value()
{
    float curr = (filtered_adc[ADC_IIN_RANK_IDX] - adc_buffer_init[ADC_IIN_RANK_IDX]) * adc_to_target_scale[ADC_IIN_RANK_IDX];
    return curr;
}

void power_source_launch()
{
    stage = 1;
}

void power_source_shutdown()
{
    stage = 4;
}

#define TICK_COUNT_VALUE (SysTick->VAL)
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

    /* init usart1 function. */
    wk_usart1_init();

    /* init usart2 function. */
    wk_usart2_init();

    /* init crc function. */
    wk_crc_init();

    /* init adc1 function. */
    // 等待ADC电源稳定，避免校准误差
    wk_delay_ms(100);
    wk_adc1_init();

    /* init tmr1 function. */
    wk_tmr1_init();

    /* init tmr15 function. */
    wk_tmr15_init();

    /* add user code begin 2 */
    // scope_init();
    ulog_init_user();
    ULOG_INFO("AT32F421 WK Demo Start");

    // 关闭所有PWM输出，避免暂态
    disable_all_output();
    // 启动定时器
    tmr_counter_enable(TMR15, TRUE);
    tmr_counter_enable(TMR1, TRUE);
    // ADC触发源使能
    tmr_channel_enable(TMR15, TMR_SELECT_CHANNEL_1, TRUE);
    // 同样的Buck置完再频率后再使能驱动PWM输出，消除暂态
    tmr_output_enable(TMR15, TRUE); // 要使能output，不然无法触发ADC采集。
    // 初始化阶段
    stage = 0;
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_HDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);
    // 延时一段时间获取ADC初始值，用于校准霍尔电流传感器
    wk_delay_ms(200);
    __disable_irq();
    // 保存初始值
    memcpy((void *)adc_buffer_init, (void *)filtered_adc, sizeof(adc_buffer_init));
    __enable_irq();

    ULOG_INFO("ADC_IIN_RANK_IDX %u", adc_buffer_init[ADC_IIN_RANK_IDX]);
    // 初始化PID控制器
    user_pid_init();
    // 设置过流保护阈值
    LLC_OC_THRESHOLD       = llc_curr_to_adc_value(LLC_INPUT_CURRENT_UPPER_LIMIT);
    LLC_Devta_OC_THRESHOLD = llc_curr_to_adc_value(LLC_INPUT_CURRENT_OC_LIMIT);
    // 初始LLC频率为最高频率
    llc_set_pwm_frequency(LLC_FREQUENCY_UPPER_LIMIT);
    // 初始LLC占空比
    llc_set_pwm_duty_cycle(0.0f);
    // 增大值Buck占空比增大输出增大，关闭设置为0
    buck_set_tmr_channel_value(0);
    // 设置完LLC的频率后再使能LLC的驱动PWM输出，消除暂态
    tmr_output_enable(TMR1, TRUE);
    llc_curr_oc_limit_adc_value_small = llc_curr_to_adc_value(0.6f);
    llc_curr_oc_limit_adc_value_small = llc_curr_oc_limit_adc_value_small << PID_SHIFT;
    llc_curr_oc_limit_adc_value_upper = llc_curr_to_adc_value(LLC_INPUT_CURRENT_OC_LIMIT);
    llc_curr_oc_limit_adc_value_upper = llc_curr_oc_limit_adc_value_upper << PID_SHIFT;
    // 设置LLC目标电压
    set_llc_volt_target_to_adc_value_q32(llc_volt_target);
    // 设置阶段为1，表示初始化完成
    power_source_launch();
    // 串口相关：数据位个数9位(包含奇偶校验位)，奇校验，1位停止位，9600波特率

    uint16_t ms_rec     = 0;
    uint16_t period_max = (LLC_PWM_PERIOD_UPPER_LIMIT + 1);
    uint16_t period_min = (0);
    uint16_t period     = period_min;

    /* add user code end 2 */
        Debug_Printf("[\r\n")
    while (1) {
        /* add user code begin 3 */
        llc_curr_freq_pid.iFmax = period << PID_SHIFT_14; // 1200放大
        wk_delay_ms(500);
        Debug_Printf("[%8d,%8d,%8d]\r\n", period, filtered_adc[ADC_IIN_RANK_IDX], adc_buffer_init[ADC_IIN_RANK_IDX])
            period++;
        if (period > period_max) {
            Debug_Printf("\r\n]")
            period = period_min;
        }
        /* add user code end 3 */
    }
}

/* add user code begin 4 */

void adc_dma_handler()
{
    static int result                  = 0;
    static uint32_t tmr_channel_value  = 0;
    static uint32_t tmr_period_value   = 0;
    static int32_t iF                  = 0;
    static int pid_stage               = 0;
    static int32_t delta_curr          = 0;
    static int32_t last_curr_adc_value = 0;

#define FILTER_SHIFT 3 // 相当于除以8的滤波系数
    tmr_output_enable(TMR15, FALSE);
    for (int i = 0; i < ADC_RANK_NUM; i++) {
        // 定点数一阶滤波: y[n] = (x[n] + 7*y[n-1]) / 8
        filtered_adc[i] = (adc_buffer[i] + ((uint32_t)filtered_adc[i] << FILTER_SHIFT) - filtered_adc[i]) >> FILTER_SHIFT;
    }
    tmr_output_enable(TMR15, TRUE);

    interrupt_cnt++;

    if (adc_buffer[ADC_IIN_RANK_IDX] > LLC_OC_THRESHOLD) {
        oc_cnt++; // 10us
        if (oc_cnt > 10) {
            // 10ms
            // LLC输入过流保护
            disable_all_output();
            protect_type = 1; // 设置保护类型为输入过流保护
        }
    } else {
        oc_cnt = 0;
    }

    // 更新PID采样值
    // 缩放见@user_pid_init
    llc_volt_pid.iSampling      = filtered_adc[ADC_V_LLC_RANK_IDX];
    llc_curr_freq_pid.iSampling = 0;

    switch (stage) {
        case 0:
            // 初始化阶段
            break;
        case 1: {
            // 使能输出，但是为最低电流
            llc_output_enable();
            interrupt_cnt = 0;
            stage         = 2;
            pid_stage     = 0;
            protect_type  = 0;
        } break;
        case 2:
            // 正常运行阶段，频率PID控制
            // 当前电压低于目标电压则会增大目标电流
            Inc_PID_Q32_Update_AddDelta(&llc_volt_pid);
            // 将电压PID的输出目标电流的对应ADC值作为频率PID的目标
            llc_curr_freq_pid.iTarget = 500;
            // 当前电流低于目标电流则会增大PERIOD寄存器值从而降低频率，使得频率靠近谐振点从而提高电流
            Inc_PID_Q32_Update_AddDelta(&llc_curr_freq_pid);
            // 超调抑制方法0：啥也不做，PID参数抑制超调，大概率是电压环的P参数过大。
            iF = llc_curr_freq_pid.iF;

            // @Apply PID
            // 将频率PID的输出目标频率的对应PERIOD寄存器值作为LLC PWM定时器的PERIOD寄存器值，默认为50%占空比
            if (iF >= (LLC_PWM_PERIOD_LOWER_LIMIT << PID_SHIFT_14)) {
                // 大于，频率低于最大频率
                tmr_period_value = (iF >> PID_SHIFT_14) - 1;
                llc_set_tmr_period(tmr_period_value);
            } else {
                // (300-0)  -> (50%-0%) -> (150-0)
                tmr_period_value_set(TMR1, LLC_PWM_PERIOD_LOWER_LIMIT);
                tmr_channel_value = ((iF) >> (PID_SHIFT_14 + 1)) + 1;
                if (interrupt_cnt & 0x01) {
                    tmr_channel_value += ((iF & (1UL << ((PID_SHIFT_14 + 1) - 1))) ? 1 : 0);
                }
                tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, tmr_channel_value);
            }
            break;
        case 4:
            // 准备停止阶段
            llc_output_disable();

            tmr_period_value_set(TMR1, LLC_PWM_PERIOD_LOWER_LIMIT);
            tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_2, 30);
            stage = 0;
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

/* add user code end 4 */
