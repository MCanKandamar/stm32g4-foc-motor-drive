/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void MX_FDCAN1_Init(void);
void MX_USB_PCD_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HAL_A_Pin GPIO_PIN_6
#define HAL_A_GPIO_Port GPIOA
#define HAL_B_Pin GPIO_PIN_7
#define HAL_B_GPIO_Port GPIOA
#define HAL_C_Pin GPIO_PIN_0
#define HAL_C_GPIO_Port GPIOB
#define CURR_C_Pin GPIO_PIN_8
#define CURR_C_GPIO_Port GPIOE
#define DRV_CS_Pin GPIO_PIN_12
#define DRV_CS_GPIO_Port GPIOB
#define SENS_C_Pin GPIO_PIN_10
#define SENS_C_GPIO_Port GPIOD
#define CURR_B_Pin GPIO_PIN_11
#define CURR_B_GPIO_Port GPIOD
#define SENS_B_Pin GPIO_PIN_12
#define SENS_B_GPIO_Port GPIOD
#define CURR_A_Pin GPIO_PIN_13
#define CURR_A_GPIO_Port GPIOD
#define SENS_A_Pin GPIO_PIN_14
#define SENS_A_GPIO_Port GPIOD
#define DRV_ENABLE_Pin GPIO_PIN_15
#define DRV_ENABLE_GPIO_Port GPIOD
#define INH_A_Pin GPIO_PIN_6
#define INH_A_GPIO_Port GPIOC
#define INH_B_Pin GPIO_PIN_7
#define INH_B_GPIO_Port GPIOC
#define INH_C_Pin GPIO_PIN_8
#define INH_C_GPIO_Port GPIOC
#define SENS_VDC_Pin GPIO_PIN_8
#define SENS_VDC_GPIO_Port GPIOA
#define INL_A_Pin GPIO_PIN_10
#define INL_A_GPIO_Port GPIOC
#define INL_B_Pin GPIO_PIN_11
#define INL_B_GPIO_Port GPIOC
#define INL_C_Pin GPIO_PIN_12
#define INL_C_GPIO_Port GPIOC
#define FDCAN_RX_Pin GPIO_PIN_0
#define FDCAN_RX_GPIO_Port GPIOD
#define FDCAN_TX_Pin GPIO_PIN_1
#define FDCAN_TX_GPIO_Port GPIOD
#define CAN_LED_Pin GPIO_PIN_2
#define CAN_LED_GPIO_Port GPIOD
#define FAULT_N_Pin GPIO_PIN_0
#define FAULT_N_GPIO_Port GPIOE
#define FAULT_N_EXTI_IRQn EXTI0_IRQn
#define PROCCESS_LED_Pin GPIO_PIN_1
#define PROCCESS_LED_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
