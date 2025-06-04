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
#define ADC_RANK_NUM 6 // ADC采样通道数
volatile uint16_t adc_buffer[ADC_RANK_NUM] = {0}; // ADC采样数据缓冲区
#define DMA1_CHANNEL1_MEMORY_BASE_ADDR ((uint32_t)adc_buffer) // DMA1通道1内存地址
#define DMA1_CHANNEL1_BUFFER_SIZE (ADC_RANK_NUM) // DMA1通道1缓冲区大小,单位是传输个数
float votlage_debug[ADC_RANK_NUM] = {0}; // 电压调试数据

#define ADC_VIN_RANK_IDX      0 // LLC输入电压,PA1
#define ADC_IO_RANK_IDX       1 // Buck输出电流,PA2
#define ADC_VO_TOTAL_RANK_IDX 2 // Buck输出电压,PA3
#define ADC_VO_MID_RANK_IDX   3 // PA6
#define ADC_IIN_RANK_IDX      4 // LLC输入电流,PA7
#define ADC_V_LLC_RANK_IDX    5 // LLC输出电压,PB2

// 预先计算的电压转换因子（Q15定点数）
#define VREF (3.3f)  // 根据实际电压修改
#define VOLTAGE_SCALE_FACTOR_Q15 (uint32_t)((VREF / 4096.0f) * 32768)  // Q15格式

// 安全限制
// 输入输出范围
// PWM频率：LLC 谐振点160KHz，100-300KHz，Buck 100KHz
// 
void LLC_Set_PWM_Frequency(uint32_t frequency) {
    // 设置LLC PWM频率
    // 计算计数值
    uint32_t count = (SystemCoreClock / frequency) - 1; // 假设SystemCoreClock是系统时钟频率
    tmr_channel_value_set(TMR15, TMR_SELECT_CHANNEL_2, count);
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

  /* init adc1 function. */
  wk_adc1_init();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* init tmr15 function. */
  wk_tmr15_init();

  /* add user code begin 2 */
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_HDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_DTERR_INT, TRUE);

  tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
  tmr_counter_enable(TMR1, TRUE);

  tmr_channel_enable(TMR15, TMR_SELECT_CHANNEL_2, TRUE);
  tmr_channel_enable(TMR15, TMR_SELECT_CHANNEL_2C, TRUE);
  tmr_counter_enable(TMR15, TRUE);
  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */

    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

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
      // Toggle gpio

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
