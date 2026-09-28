/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, STAT1_LED_GPIO_OUT_Pin|WARN_LED_GPIO_OUT_Pin|STAT2_LED_GPIO_OUT_Pin|ERR_LED_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, uC_GPIO_OUT_AIR_POS_COMMAND_Pin|uC_GPIO_OUT_AIR_NEG_COMMAND_Pin|uC_GPIO_OUT_STEF01FTR_Enable_Pin|SDC_GENERIC_SWITCH_GPIO_OUT_Pin
                          |uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin|L9963TH_TXEN_GPIO_OUT_Pin|L9963TH_ISOFREQ_GPIO_OUT_Pin|M95256_nS_GPIO_OUT_Pin
                          |M95256_nW_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AMS_ERROR_Pin|L9963TL_ISOFREQ_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L9963TL_NCS_GPIO_OUT_Pin|L9963TL_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STAT1_LED_GPIO_OUT_Pin WARN_LED_GPIO_OUT_Pin STAT2_LED_GPIO_OUT_Pin ERR_LED_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = STAT1_LED_GPIO_OUT_Pin|WARN_LED_GPIO_OUT_Pin|STAT2_LED_GPIO_OUT_Pin|ERR_LED_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIR_NEG_INT_STATE_CLOSED_3V3_Pin AIR_POS_MECH_STATE_OPEN_3V3_Pin FB_TSAL_GREEN_3V3_Pin FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_Pin */
  GPIO_InitStruct.Pin = AIR_NEG_INT_STATE_CLOSED_3V3_Pin|AIR_POS_MECH_STATE_OPEN_3V3_Pin|FB_TSAL_GREEN_3V3_Pin|FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STEF01FTR_Fault_Feedback_3V3_Pin AIR_POS_INT_STATE_CLOSED_3V3_Pin L9963TL_BNE_GPIO_IN_Pin */
  GPIO_InitStruct.Pin = STEF01FTR_Fault_Feedback_3V3_Pin|AIR_POS_INT_STATE_CLOSED_3V3_Pin|L9963TL_BNE_GPIO_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : nSTG_PRECH_3V3_Pin nSTG_AIR_POS_3V3_Pin AIR_NEG_MECH_STATE_OPEN_3V3_Pin FB_IMPLAUSIBILITY_3V3_Pin */
  GPIO_InitStruct.Pin = nSTG_PRECH_3V3_Pin|nSTG_AIR_POS_3V3_Pin|AIR_NEG_MECH_STATE_OPEN_3V3_Pin|FB_IMPLAUSIBILITY_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FB_DCBUS_IMPLAUSIBILITY_3V3_Pin FB_DCBUS_IMPLAUSIBILITY_3V3E9_Pin AMS_ERROR_LATCHED_3V3_Pin DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_Pin
                           IMD_ERROR_LATCHED_3V3_Pin FB_SDC_to_BMS_INPUT_2_3V3_Pin IMD_MEASUREMENT_3V3_Pin */
  GPIO_InitStruct.Pin = FB_DCBUS_IMPLAUSIBILITY_3V3_Pin|FB_DCBUS_IMPLAUSIBILITY_3V3E9_Pin|AMS_ERROR_LATCHED_3V3_Pin|DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_Pin
                          |IMD_ERROR_LATCHED_3V3_Pin|FB_SDC_to_BMS_INPUT_2_3V3_Pin|IMD_MEASUREMENT_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : IMD_STATUS_3V3_Pin */
  GPIO_InitStruct.Pin = IMD_STATUS_3V3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IMD_STATUS_3V3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nSTG_AIR_NEG_3V3_Pin nDCBUS_OVER_60V_3V3_Pin nSTG_DCBUS_OVER60_3V3_Pin FAULT_LINE_1_GPIO_IN_Pin
                           L9963TH_BNE_GPIO_IN_Pin */
  GPIO_InitStruct.Pin = nSTG_AIR_NEG_3V3_Pin|nDCBUS_OVER_60V_3V3_Pin|nSTG_DCBUS_OVER60_3V3_Pin|FAULT_LINE_1_GPIO_IN_Pin
                          |L9963TH_BNE_GPIO_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : uC_GPIO_OUT_AIR_POS_COMMAND_Pin uC_GPIO_OUT_AIR_NEG_COMMAND_Pin uC_GPIO_OUT_STEF01FTR_Enable_Pin SDC_GENERIC_SWITCH_GPIO_OUT_Pin
                           uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin */
  GPIO_InitStruct.Pin = uC_GPIO_OUT_AIR_POS_COMMAND_Pin|uC_GPIO_OUT_AIR_NEG_COMMAND_Pin|uC_GPIO_OUT_STEF01FTR_Enable_Pin|SDC_GENERIC_SWITCH_GPIO_OUT_Pin
                          |uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : AMS_ERROR_Pin */
  GPIO_InitStruct.Pin = AMS_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMS_ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L9963TL_ISOFREQ_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = L9963TL_ISOFREQ_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L9963TL_ISOFREQ_GPIO_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L9963TL_DIS_GPIO_INOUT_Pin */
  GPIO_InitStruct.Pin = L9963TL_DIS_GPIO_INOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L9963TL_DIS_GPIO_INOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L9963TL_NCS_GPIO_OUT_Pin L9963TL_TXEN_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = L9963TL_NCS_GPIO_OUT_Pin|L9963TL_TXEN_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L9963TH_TXEN_GPIO_OUT_Pin L9963TH_ISOFREQ_GPIO_OUT_Pin M95256_nS_GPIO_OUT_Pin M95256_nW_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = L9963TH_TXEN_GPIO_OUT_Pin|L9963TH_ISOFREQ_GPIO_OUT_Pin|M95256_nS_GPIO_OUT_Pin|M95256_nW_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : L9963TH_DIS_GPIO_INOUT_Pin */
  GPIO_InitStruct.Pin = L9963TH_DIS_GPIO_INOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L9963TH_DIS_GPIO_INOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L9963TH_NCS_GPIO_OUT_Pin */
  GPIO_InitStruct.Pin = L9963TH_NCS_GPIO_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L9963TH_NCS_GPIO_OUT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
