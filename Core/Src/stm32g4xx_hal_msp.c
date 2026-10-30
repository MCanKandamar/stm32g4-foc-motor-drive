/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32g4xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_adc3;

extern DMA_HandleTypeDef hdma_adc4;

extern DMA_HandleTypeDef hdma_adc5;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                    /**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
  */
  HAL_PWREx_DisableUCPDDeadBattery();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_ADC345_CLK_ENABLED=0;

/**
  * @brief ADC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hadc->Instance==ADC3)
  {
    /* USER CODE BEGIN ADC3_MspInit 0 */

    /* USER CODE END ADC3_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_ADC345_CLK_ENABLED++;
    if(HAL_RCC_ADC345_CLK_ENABLED==1){
      __HAL_RCC_ADC345_CLK_ENABLE();
    }

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PE8     ------> ADC3_IN6
    PD11     ------> ADC3_IN8
    PD13     ------> ADC3_IN10
    */
    GPIO_InitStruct.Pin = CURR_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CURR_C_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CURR_B_Pin|CURR_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* ADC3 DMA Init */
    /* ADC3 Init */
    hdma_adc3.Instance = DMA1_Channel3;
    hdma_adc3.Init.Request = DMA_REQUEST_ADC3;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc3);

    /* ADC3 interrupt Init */
    HAL_NVIC_SetPriority(ADC3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC3_IRQn);
    /* USER CODE BEGIN ADC3_MspInit 1 */

    /* USER CODE END ADC3_MspInit 1 */
  }
  else if(hadc->Instance==ADC4)
  {
    /* USER CODE BEGIN ADC4_MspInit 0 */

    /* USER CODE END ADC4_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_ADC345_CLK_ENABLED++;
    if(HAL_RCC_ADC345_CLK_ENABLED==1){
      __HAL_RCC_ADC345_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**ADC4 GPIO Configuration
    PD10     ------> ADC4_IN7
    PD12     ------> ADC4_IN9
    PD14     ------> ADC4_IN11
    */
    GPIO_InitStruct.Pin = SENS_C_Pin|SENS_B_Pin|SENS_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* ADC4 DMA Init */
    /* ADC4 Init */
    hdma_adc4.Instance = DMA1_Channel1;
    hdma_adc4.Init.Request = DMA_REQUEST_ADC4;
    hdma_adc4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc4.Init.Mode = DMA_CIRCULAR;
    hdma_adc4.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc4);

    /* USER CODE BEGIN ADC4_MspInit 1 */

    /* USER CODE END ADC4_MspInit 1 */
  }
  else if(hadc->Instance==ADC5)
  {
    /* USER CODE BEGIN ADC5_MspInit 0 */

    /* USER CODE END ADC5_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC345;
    PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    HAL_RCC_ADC345_CLK_ENABLED++;
    if(HAL_RCC_ADC345_CLK_ENABLED==1){
      __HAL_RCC_ADC345_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC5 GPIO Configuration
    PA8     ------> ADC5_IN1
    */
    GPIO_InitStruct.Pin = SENS_VDC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SENS_VDC_GPIO_Port, &GPIO_InitStruct);

    /* ADC5 DMA Init */
    /* ADC5 Init */
    hdma_adc5.Instance = DMA1_Channel2;
    hdma_adc5.Init.Request = DMA_REQUEST_ADC5;
    hdma_adc5.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc5.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc5.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc5.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc5.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc5.Init.Mode = DMA_CIRCULAR;
    hdma_adc5.Init.Priority = DMA_PRIORITY_MEDIUM;
    if (HAL_DMA_Init(&hdma_adc5) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc5);

    /* USER CODE BEGIN ADC5_MspInit 1 */

    /* USER CODE END ADC5_MspInit 1 */
  }

}

/**
  * @brief ADC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC3)
  {
    /* USER CODE BEGIN ADC3_MspDeInit 0 */

    /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC345_CLK_ENABLED--;
    if(HAL_RCC_ADC345_CLK_ENABLED==0){
      __HAL_RCC_ADC345_CLK_DISABLE();
    }

    /**ADC3 GPIO Configuration
    PE8     ------> ADC3_IN6
    PD11     ------> ADC3_IN8
    PD13     ------> ADC3_IN10
    */
    HAL_GPIO_DeInit(CURR_C_GPIO_Port, CURR_C_Pin);

    HAL_GPIO_DeInit(GPIOD, CURR_B_Pin|CURR_A_Pin);

    /* ADC3 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* ADC3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(ADC3_IRQn);
    /* USER CODE BEGIN ADC3_MspDeInit 1 */

    /* USER CODE END ADC3_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC4)
  {
    /* USER CODE BEGIN ADC4_MspDeInit 0 */

    /* USER CODE END ADC4_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC345_CLK_ENABLED--;
    if(HAL_RCC_ADC345_CLK_ENABLED==0){
      __HAL_RCC_ADC345_CLK_DISABLE();
    }

    /**ADC4 GPIO Configuration
    PD10     ------> ADC4_IN7
    PD12     ------> ADC4_IN9
    PD14     ------> ADC4_IN11
    */
    HAL_GPIO_DeInit(GPIOD, SENS_C_Pin|SENS_B_Pin|SENS_A_Pin);

    /* ADC4 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    /* USER CODE BEGIN ADC4_MspDeInit 1 */

    /* USER CODE END ADC4_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC5)
  {
    /* USER CODE BEGIN ADC5_MspDeInit 0 */

    /* USER CODE END ADC5_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC345_CLK_ENABLED--;
    if(HAL_RCC_ADC345_CLK_ENABLED==0){
      __HAL_RCC_ADC345_CLK_DISABLE();
    }

    /**ADC5 GPIO Configuration
    PA8     ------> ADC5_IN1
    */
    HAL_GPIO_DeInit(SENS_VDC_GPIO_Port, SENS_VDC_Pin);

    /* ADC5 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    /* USER CODE BEGIN ADC5_MspDeInit 1 */

    /* USER CODE END ADC5_MspDeInit 1 */
  }

}

/**
  * @brief CORDIC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hcordic: CORDIC handle pointer
  * @retval None
  */
void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
    /* USER CODE BEGIN CORDIC_MspInit 0 */

    /* USER CODE END CORDIC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();
    /* USER CODE BEGIN CORDIC_MspInit 1 */

    /* USER CODE END CORDIC_MspInit 1 */

  }

}

/**
  * @brief CORDIC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hcordic: CORDIC handle pointer
  * @retval None
  */
void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* hcordic)
{
  if(hcordic->Instance==CORDIC)
  {
    /* USER CODE BEGIN CORDIC_MspDeInit 0 */

    /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();
    /* USER CODE BEGIN CORDIC_MspDeInit 1 */

    /* USER CODE END CORDIC_MspDeInit 1 */
  }

}

/**
  * @brief FDCAN MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hfdcan: FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hfdcan->Instance==FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspInit 0 */

    /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = FDCAN_RX_Pin|FDCAN_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspInit 1 */

    /* USER CODE END FDCAN1_MspInit 1 */

  }

}

/**
  * @brief FDCAN MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hfdcan: FDCAN handle pointer
  * @retval None
  */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
  if(hfdcan->Instance==FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

    /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, FDCAN_RX_Pin|FDCAN_TX_Pin);

    /* FDCAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

    /* USER CODE END FDCAN1_MspDeInit 1 */
  }

}

/**
  * @brief SPI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspInit 0 */

    /* USER CODE END SPI2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI2_MspInit 1 */

    /* USER CODE END SPI2_MspInit 1 */

  }

}

/**
  * @brief SPI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{
  if(hspi->Instance==SPI2)
  {
    /* USER CODE BEGIN SPI2_MspDeInit 0 */

    /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* USER CODE BEGIN SPI2_MspDeInit 1 */

    /* USER CODE END SPI2_MspDeInit 1 */
  }

}

/**
  * @brief TIM_Base MSP Initialization
  * This function configures the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    */
    GPIO_InitStruct.Pin = HAL_A_Pin|HAL_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = HAL_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(HAL_C_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
  }
  else if(htim_base->Instance==TIM8)
  {
    /* USER CODE BEGIN TIM8_MspInit 0 */

    /* USER CODE END TIM8_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();
    /* USER CODE BEGIN TIM8_MspInit 1 */

    /* USER CODE END TIM8_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM8)
  {
    /* USER CODE BEGIN TIM8_MspPostInit 0 */

    /* USER CODE END TIM8_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM8 GPIO Configuration
    PC6     ------> TIM8_CH1
    PC7     ------> TIM8_CH2
    PC8     ------> TIM8_CH3
    PC10     ------> TIM8_CH1N
    PC11     ------> TIM8_CH2N
    PC12     ------> TIM8_CH3N
    */
    GPIO_InitStruct.Pin = INH_A_Pin|INH_B_Pin|INH_C_Pin|INL_A_Pin
                          |INL_B_Pin|INL_C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM8_MspPostInit 1 */

    /* USER CODE END TIM8_MspPostInit 1 */
  }

}
/**
  * @brief TIM_Base MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param htim_base: TIM_Base handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM3)
  {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */

    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    PB0     ------> TIM3_CH3
    */
    HAL_GPIO_DeInit(GPIOA, HAL_A_Pin|HAL_B_Pin);

    HAL_GPIO_DeInit(HAL_C_GPIO_Port, HAL_C_Pin);

    /* USER CODE BEGIN TIM3_MspDeInit 1 */

    /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM8)
  {
    /* USER CODE BEGIN TIM8_MspDeInit 0 */

    /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();
    /* USER CODE BEGIN TIM8_MspDeInit 1 */

    /* USER CODE END TIM8_MspDeInit 1 */
  }

}

/**
  * @brief PCD MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
void HAL_PCD_MspInit(PCD_HandleTypeDef* hpcd)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(hpcd->Instance==USB)
  {
    /* USER CODE BEGIN USB_MspInit 0 */

    /* USER CODE END USB_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_USB_CLK_ENABLE();
    /* USB interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_HP_IRQn);
    HAL_NVIC_SetPriority(USB_LP_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
    /* USER CODE BEGIN USB_MspInit 1 */

    /* USER CODE END USB_MspInit 1 */

  }

}

/**
  * @brief PCD MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hpcd: PCD handle pointer
  * @retval None
  */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef* hpcd)
{
  if(hpcd->Instance==USB)
  {
    /* USER CODE BEGIN USB_MspDeInit 0 */

    /* USER CODE END USB_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USB_CLK_DISABLE();

    /* USB interrupt DeInit */
    HAL_NVIC_DisableIRQ(USB_HP_IRQn);
    HAL_NVIC_DisableIRQ(USB_LP_IRQn);
    /* USER CODE BEGIN USB_MspDeInit 1 */

    /* USER CODE END USB_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
