/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define STAT1_LED_GPIO_OUT_Pin GPIO_PIN_2
#define STAT1_LED_GPIO_OUT_GPIO_Port GPIOE
#define WARN_LED_GPIO_OUT_Pin GPIO_PIN_3
#define WARN_LED_GPIO_OUT_GPIO_Port GPIOE
#define STAT2_LED_GPIO_OUT_Pin GPIO_PIN_4
#define STAT2_LED_GPIO_OUT_GPIO_Port GPIOE
#define ERR_LED_GPIO_OUT_Pin GPIO_PIN_5
#define ERR_LED_GPIO_OUT_GPIO_Port GPIOE
#define AIR_NEG_INT_STATE_CLOSED_3V3_Pin GPIO_PIN_0
#define AIR_NEG_INT_STATE_CLOSED_3V3_GPIO_Port GPIOC
#define AIR_POS_MECH_STATE_OPEN_3V3_Pin GPIO_PIN_1
#define AIR_POS_MECH_STATE_OPEN_3V3_GPIO_Port GPIOC
#define FB_TSAL_GREEN_3V3_Pin GPIO_PIN_2
#define FB_TSAL_GREEN_3V3_GPIO_Port GPIOC
#define FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_Pin GPIO_PIN_3
#define FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_GPIO_Port GPIOC
#define STEF01FTR_Fault_Feedback_3V3_Pin GPIO_PIN_0
#define STEF01FTR_Fault_Feedback_3V3_GPIO_Port GPIOA
#define FB_SDC_TO_AIRs_Pin GPIO_PIN_2
#define FB_SDC_TO_AIRs_GPIO_Port GPIOA
#define FB_SDC_POST_GEN_SWITCH_Pin GPIO_PIN_3
#define FB_SDC_POST_GEN_SWITCH_GPIO_Port GPIOA
#define AIR_POS_INT_STATE_CLOSED_3V3_Pin GPIO_PIN_4
#define AIR_POS_INT_STATE_CLOSED_3V3_GPIO_Port GPIOA
#define M95256_C_SPI1_SCK_Pin GPIO_PIN_5
#define M95256_C_SPI1_SCK_GPIO_Port GPIOA
#define M95256_Q_SPI1_MISO_Pin GPIO_PIN_6
#define M95256_Q_SPI1_MISO_GPIO_Port GPIOA
#define M95256_D_SPI1_MOSI_Pin GPIO_PIN_7
#define M95256_D_SPI1_MOSI_GPIO_Port GPIOA
#define FB_SDC_POST_IMD_SWITCH_Pin GPIO_PIN_4
#define FB_SDC_POST_IMD_SWITCH_GPIO_Port GPIOC
#define FB_SDC_POST_AMS_SWITCH_Pin GPIO_PIN_0
#define FB_SDC_POST_AMS_SWITCH_GPIO_Port GPIOB
#define FB_SDC_to_BMS_INPUT_1_Pin GPIO_PIN_1
#define FB_SDC_to_BMS_INPUT_1_GPIO_Port GPIOB
#define nSTG_PRECH_3V3_Pin GPIO_PIN_2
#define nSTG_PRECH_3V3_GPIO_Port GPIOB
#define FB_DCBUS_IMPLAUSIBILITY_3V3_Pin GPIO_PIN_7
#define FB_DCBUS_IMPLAUSIBILITY_3V3_GPIO_Port GPIOE
#define FB_DCBUS_IMPLAUSIBILITY_3V3E9_Pin GPIO_PIN_9
#define FB_DCBUS_IMPLAUSIBILITY_3V3E9_GPIO_Port GPIOE
#define AMS_ERROR_LATCHED_3V3_Pin GPIO_PIN_10
#define AMS_ERROR_LATCHED_3V3_GPIO_Port GPIOE
#define DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_Pin GPIO_PIN_11
#define DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_GPIO_Port GPIOE
#define IMD_STATUS_3V3_Pin GPIO_PIN_12
#define IMD_STATUS_3V3_GPIO_Port GPIOE
#define IMD_ERROR_LATCHED_3V3_Pin GPIO_PIN_13
#define IMD_ERROR_LATCHED_3V3_GPIO_Port GPIOE
#define FB_SDC_to_BMS_INPUT_2_3V3_Pin GPIO_PIN_14
#define FB_SDC_to_BMS_INPUT_2_3V3_GPIO_Port GPIOE
#define IMD_MEASUREMENT_3V3_Pin GPIO_PIN_15
#define IMD_MEASUREMENT_3V3_GPIO_Port GPIOE
#define nSTG_AIR_NEG_3V3_Pin GPIO_PIN_8
#define nSTG_AIR_NEG_3V3_GPIO_Port GPIOD
#define nDCBUS_OVER_60V_3V3_Pin GPIO_PIN_9
#define nDCBUS_OVER_60V_3V3_GPIO_Port GPIOD
#define nSTG_DCBUS_OVER60_3V3_Pin GPIO_PIN_10
#define nSTG_DCBUS_OVER60_3V3_GPIO_Port GPIOD
#define uC_GPIO_OUT_AIR_POS_COMMAND_Pin GPIO_PIN_11
#define uC_GPIO_OUT_AIR_POS_COMMAND_GPIO_Port GPIOD
#define uC_GPIO_OUT_AIR_NEG_COMMAND_Pin GPIO_PIN_12
#define uC_GPIO_OUT_AIR_NEG_COMMAND_GPIO_Port GPIOD
#define uC_GPIO_OUT_STEF01FTR_Enable_Pin GPIO_PIN_13
#define uC_GPIO_OUT_STEF01FTR_Enable_GPIO_Port GPIOD
#define SDC_GENERIC_SWITCH_GPIO_OUT_Pin GPIO_PIN_14
#define SDC_GENERIC_SWITCH_GPIO_OUT_GPIO_Port GPIOD
#define uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin GPIO_PIN_15
#define uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_GPIO_Port GPIOD
#define AMS_ERROR_Pin GPIO_PIN_6
#define AMS_ERROR_GPIO_Port GPIOC
#define L9963TL_ISOFREQ_GPIO_OUT_Pin GPIO_PIN_8
#define L9963TL_ISOFREQ_GPIO_OUT_GPIO_Port GPIOC
#define L9963TL_DIS_GPIO_INOUT_Pin GPIO_PIN_9
#define L9963TL_DIS_GPIO_INOUT_GPIO_Port GPIOC
#define L9963TL_NCS_GPIO_OUT_Pin GPIO_PIN_8
#define L9963TL_NCS_GPIO_OUT_GPIO_Port GPIOA
#define L9963TL_BNE_GPIO_IN_Pin GPIO_PIN_9
#define L9963TL_BNE_GPIO_IN_GPIO_Port GPIOA
#define L9963TL_TXEN_GPIO_OUT_Pin GPIO_PIN_10
#define L9963TL_TXEN_GPIO_OUT_GPIO_Port GPIOA
#define FAULT_LINE_1_GPIO_IN_Pin GPIO_PIN_0
#define FAULT_LINE_1_GPIO_IN_GPIO_Port GPIOD
#define L9963TH_TXEN_GPIO_OUT_Pin GPIO_PIN_1
#define L9963TH_TXEN_GPIO_OUT_GPIO_Port GPIOD
#define L9963TH_ISOFREQ_GPIO_OUT_Pin GPIO_PIN_2
#define L9963TH_ISOFREQ_GPIO_OUT_GPIO_Port GPIOD
#define L9963TH_BNE_GPIO_IN_Pin GPIO_PIN_3
#define L9963TH_BNE_GPIO_IN_GPIO_Port GPIOD
#define L9963TH_DIS_GPIO_INOUT_Pin GPIO_PIN_4
#define L9963TH_DIS_GPIO_INOUT_GPIO_Port GPIOD
#define M95256_nS_GPIO_OUT_Pin GPIO_PIN_5
#define M95256_nS_GPIO_OUT_GPIO_Port GPIOD
#define M95256_nW_GPIO_OUT_Pin GPIO_PIN_6
#define M95256_nW_GPIO_OUT_GPIO_Port GPIOD
#define L9963TH_NCS_GPIO_OUT_Pin GPIO_PIN_5
#define L9963TH_NCS_GPIO_OUT_GPIO_Port GPIOB
#define nSTG_AIR_POS_3V3_Pin GPIO_PIN_7
#define nSTG_AIR_POS_3V3_GPIO_Port GPIOB
#define AIR_NEG_MECH_STATE_OPEN_3V3_Pin GPIO_PIN_8
#define AIR_NEG_MECH_STATE_OPEN_3V3_GPIO_Port GPIOB
#define FB_IMPLAUSIBILITY_3V3_Pin GPIO_PIN_9
#define FB_IMPLAUSIBILITY_3V3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* PRECHARGE COMMAND MACROS */
#define Close_Precharge() \
    HAL_GPIO_WritePin(    \
        uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_GPIO_Port, uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin, GPIO_PIN_SET)
#define Open_Precharge() \
    HAL_GPIO_WritePin(   \
        uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_GPIO_Port, uC_GPIO_OUT_DCBUS_PRCH_RLY_COMMAND_Pin, GPIO_PIN_RESET)
/* AIRs COMMAND MACROS */
#define Close_Air_Pos() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_POS_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_POS_COMMAND_Pin, GPIO_PIN_SET)
#define Open_Air_Pos() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_POS_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_POS_COMMAND_Pin, GPIO_PIN_RESET)
#define Close_Air_Neg() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_NEG_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_NEG_COMMAND_Pin, GPIO_PIN_SET)
#define Open_Air_Neg() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_AIR_NEG_COMMAND_GPIO_Port, uC_GPIO_OUT_AIR_NEG_COMMAND_Pin, GPIO_PIN_RESET)
/* AMS ERROR MACROS */
#define Set_AMS_Error()   HAL_GPIO_WritePin(AMS_ERROR_GPIO_Port, AMS_ERROR_Pin, GPIO_PIN_SET)
#define Reset_AMS_Error() HAL_GPIO_WritePin(AMS_ERROR_GPIO_Port, AMS_ERROR_Pin, GPIO_PIN_RESET)
/* LED MACROS */
#define Warn_LED_On()   HAL_GPIO_WritePin(WARN_LED_GPIO_OUT_GPIO_Port, WARN_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Warn_LED_Off()  HAL_GPIO_WritePin(WARN_LED_GPIO_OUT_GPIO_Port, WARN_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define Stat1_LED_On()  HAL_GPIO_WritePin(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Stat1_LED_Off() HAL_GPIO_WritePin(STAT1_LED_GPIO_OUT_GPIO_Port, STAT1_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define Stat2_LED_On()  HAL_GPIO_WritePin(STAT2_LED_GPIO_OUT_GPIO_Port, STAT2_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Stat2_LED_Off() HAL_GPIO_WritePin(STAT2_LED_GPIO_OUT_GPIO_Port, STAT2_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define Err_LED_On()    HAL_GPIO_WritePin(ERR_LED_GPIO_OUT_GPIO_Port, ERR_LED_GPIO_OUT_Pin, GPIO_PIN_SET)
#define Err_LED_Off()   HAL_GPIO_WritePin(ERR_LED_GPIO_OUT_GPIO_Port, ERR_LED_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* NCS of L9963 H an L */
#define L9963TH_NCS_High() HAL_GPIO_WritePin(L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TH_NCS_Low()  HAL_GPIO_WritePin(L9963TH_NCS_GPIO_OUT_GPIO_Port, L9963TH_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define L9963TL_NCS_High() HAL_GPIO_WritePin(L9963TL_NCS_GPIO_OUT_GPIO_Port, L9963TL_NCS_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TL_NCS_Low()  HAL_GPIO_WritePin(L9963TL_NCS_GPIO_OUT_GPIO_Port, L9963TL_NCS_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* TXEN of L9963 H an L */
#define L9963TH_TXEN_En()  HAL_GPIO_WritePin(L9963TH_TXEN_GPIO_OUT_GPIO_Port, L9963TH_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TH_TXEN_Dis() HAL_GPIO_WritePin(L9963TH_TXEN_GPIO_OUT_GPIO_Port, L9963TH_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET)
#define L9963TL_TXEN_En()  HAL_GPIO_WritePin(L9963TL_TXEN_GPIO_OUT_GPIO_Port, L9963TL_TXEN_GPIO_OUT_Pin, GPIO_PIN_SET)
#define L9963TL_TXEN_Dis() HAL_GPIO_WritePin(L9963TL_TXEN_GPIO_OUT_GPIO_Port, L9963TL_TXEN_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* BNE of L9963 H and L (Bus Not Empty Pin)*/
#define L9963TH_BNE() ((uint8_t)HAL_GPIO_ReadPin(L9963TH_BNE_GPIO_IN_GPIO_Port, L9963TH_BNE_GPIO_IN_Pin))
#define L9963TL_BNE() ((uint8_t)HAL_GPIO_ReadPin(L9963TL_BNE_GPIO_IN_GPIO_Port, L9963TL_BNE_GPIO_IN_Pin))
/* DIS pin of L9963T H and L*/
#define L9963TH_DIS_SET() HAL_GPIO_WritePin(L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin, GPIO_PIN_SET)
#define L9963TH_DIS_RESET() \
    HAL_GPIO_WritePin(L9963TH_DIS_GPIO_INOUT_GPIO_Port, L9963TH_DIS_GPIO_INOUT_Pin, GPIO_PIN_RESET)
#define L9963TL_DIS_SET() HAL_GPIO_WritePin(L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin, GPIO_PIN_SET)
#define L9963TL_DIS_RESET() \
    HAL_GPIO_WritePin(L9963TL_DIS_GPIO_INOUT_GPIO_Port, L9963TL_DIS_GPIO_INOUT_Pin, GPIO_PIN_RESET)
/* Shutdown Circuit Activation (SDC)*/
#define SDC_On() HAL_GPIO_WritePin(SDC_GENERIC_SWITCH_GPIO_OUT_GPIO_Port, SDC_GENERIC_SWITCH_GPIO_OUT_Pin, GPIO_PIN_SET)

#define SDC_Off() \
    HAL_GPIO_WritePin(SDC_GENERIC_SWITCH_GPIO_OUT_GPIO_Port, SDC_GENERIC_SWITCH_GPIO_OUT_Pin, GPIO_PIN_RESET)
/* AIRs signals */
#define AIRs_Pos_Int_Closed() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_POS_INT_STATE_CLOSED_3V3_GPIO_Port, AIR_POS_INT_STATE_CLOSED_3V3_Pin))
#define AIRs_Neg_Int_Closed() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_NEG_INT_STATE_CLOSED_3V3_GPIO_Port, AIR_NEG_INT_STATE_CLOSED_3V3_Pin))
#define AIRs_Pos_Mech_Open() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_POS_MECH_STATE_OPEN_3V3_GPIO_Port, AIR_POS_MECH_STATE_OPEN_3V3_Pin))
#define AIRs_Neg_Mech_Open() \
    ((uint8_t)HAL_GPIO_ReadPin(AIR_NEG_MECH_STATE_OPEN_3V3_GPIO_Port, AIR_NEG_MECH_STATE_OPEN_3V3_Pin))
/* Fuse Enable STEF*/
#define STEF01FTR_En() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_STEF01FTR_Enable_GPIO_Port, uC_GPIO_OUT_STEF01FTR_Enable_Pin, GPIO_PIN_SET)
#define STEF01FTR_Dis() \
    HAL_GPIO_WritePin(uC_GPIO_OUT_STEF01FTR_Enable_GPIO_Port, uC_GPIO_OUT_STEF01FTR_Enable_Pin, GPIO_PIN_RESET)
/* DC BUS signals*/
#define DCBUS_Over60V()     ((uint8_t)HAL_GPIO_ReadPin(nDCBUS_OVER_60V_3V3_GPIO_Port, nDCBUS_OVER_60V_3V3_Pin))
#define STG_DCBUS_Over60V() ((uint8_t)HAL_GPIO_ReadPin(nSTG_DCBUS_OVER60_3V3_GPIO_Port, nSTG_DCBUS_OVER60_3V3_Pin))
#define PRCH_closed()       ((uint8_t)HAL_GPIO_ReadPin(DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_GPIO_Port, DCBUS_PRCH_RLY_INT_STATE_CLOSED_3V3_Pin))
#define DCBUS_Rly_Implausibility() ((uint8_t)HAL_GPIO_ReadPin(FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_GPIO_Port, FB_DCBUS_RLY_IMPLAUSIBILITY_3V3_Pin))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
