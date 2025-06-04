/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_pwc.c
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
#include "wk_pwc.h"

/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief  init pwc function.
  * @param  none
  * @retval none
  */
void wk_pwc_init(void)
{
  /* add user code begin pwc_init 0 */

  /* add user code end pwc_init 0 */

  exint_init_type exint_init_struct;

  /* add user code begin pwc_init 1 */

  /* add user code end pwc_init 1 */

  /* set the threshold voltage*/
  pwc_pvm_level_select(PWC_PVM_VOLTAGE_2V4);
  /* enable power voltage monitor */
  pwc_power_voltage_monitor_enable(TRUE);

  exint_default_para_init(&exint_init_struct);
  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
  exint_init_struct.line_select = EXINT_LINE_16;
  exint_init_struct.line_polarity = EXINT_TRIGGER_BOTH_EDGE;
  exint_init(&exint_init_struct);
  /**
   * Users need to configure PWC interrupt functions according to the actual application.
   * 1. Add the user's interrupt handler code into the below function in the at32f421_int.c file.
   *     --void PVM_IRQHandler (void)
   */

  /* add user code begin pwc_init 2 */

  /* add user code end pwc_init 2 */

  /* when power voltage monitor enable,exint line16 flag will be set */
  while(exint_flag_get(EXINT_LINE_16) == RESET);

  /* clear the unexpected exint line flag */
  exint_flag_clear(EXINT_LINE_16);

  /* clear the unexpected nvic pending flag */
  NVIC_ClearPendingIRQ(PVM_IRQn);

  /* add user code begin pwc_init 3 */

  /* add user code end pwc_init 3 */
}

/* add user code begin 1 */

/* add user code end 1 */
