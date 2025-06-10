/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_usart.c
  * @brief    work bench config program
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
#include "wk_usart.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init usart2 function
  * @param  none
  * @retval none
  */
void wk_usart2_init(void)
{
  /* add user code begin usart2_init 0 */

  /* add user code end usart2_init 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin usart2_init 1 */

  /* add user code end usart2_init 1 */

  /* configure the TX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE8, GPIO_MUX_4);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_8;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the RX pin */
  gpio_pin_mux_config(GPIOA, GPIO_PINS_SOURCE15, GPIO_MUX_1);
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_15;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure param */
  usart_init(USART2, 115200, USART_DATA_9BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART2, TRUE);
  usart_receiver_enable(USART2, TRUE);
  usart_parity_selection_config(USART2, USART_PARITY_ODD);

  usart_hardware_flow_control_set(USART2, USART_HARDWARE_FLOW_NONE);

  /**
   * Users need to configure USART2 interrupt functions according to the actual application.
   * 1. Call the below function to enable the corresponding USART2 interrupt.
   *     --usart_interrupt_enable(...)
   * 2. Add the user's interrupt handler code into the below function in the at32f421_int.c file.
   *     --void USART2_IRQHandler(void)
   */

  /* add user code begin usart2_init 2 */

  /* add user code end usart2_init 2 */
  
  usart_enable(USART2, TRUE);

  /* add user code begin usart2_init 3 */

  /* add user code end usart2_init 3 */
}

/* add user code begin 1 */

/* add user code end 1 */
