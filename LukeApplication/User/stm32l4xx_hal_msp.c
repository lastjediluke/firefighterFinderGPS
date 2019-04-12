/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @author  MCD Application Team
  * @brief   MSP Initialization and de-Initialization.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/**
  * Initializes the Global MSP.
  */

 void Periph_Config(void)
 {
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  PeriphClkInit.PeriphClockSelection  = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                                       |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C2
                                       |RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart1ClockSelection  = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection  = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection    = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.RngClockSelection     = RCC_RNGCLKSOURCE_MSI;

  PeriphClkInit.RTCClockSelection     = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

   __HAL_RCC_PWR_CLK_ENABLE();

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}


void HAL_RNG_MspInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
      /* Peripheral clock enable */
    __HAL_RCC_RNG_CLK_ENABLE();
  }
}

void HAL_RNG_MspDeInit(RNG_HandleTypeDef* hrng)
{
  if(hrng->Instance==RNG)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RNG_CLK_DISABLE();
  }
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    __HAL_RCC_RTC_ENABLE();
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{
  if(hrtc->Instance==RTC)
  {
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  }
}

//// Zj
///**
//* @brief UART MSP Initialization
//* This function configures the hardware resources used in this example
//* @param huart: UART handle pointer
//* @retval None
//*/
//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(huart->Instance==UART4)
//  {
//  /* USER CODE BEGIN UART4_MspInit 0 */
//
//  /* USER CODE END UART4_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_UART4_CLK_ENABLE();
//
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    /**UART4 GPIO Configuration
//    PA0     ------> UART4_TX
//    PA1     ------> UART4_RX
//    */
//    GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN UART4_MspInit 1 */
//
//  /* USER CODE END UART4_MspInit 1 */
//  }
//  else if(huart->Instance==USART1)
//  {
//  /* USER CODE BEGIN USART1_MspInit 0 */
//
//  /* USER CODE END USART1_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_USART1_CLK_ENABLE();
//
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    /**USART1 GPIO Configuration
//    PB6     ------> USART1_TX
//    PB7     ------> USART1_RX
//    */
//    GPIO_InitStruct.Pin = ST_LINK_UART1_TX_Pin|ST_LINK_UART1_RX_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN USART1_MspInit 1 */
//
//  /* USER CODE END USART1_MspInit 1 */
//  }
//  else if(huart->Instance==USART3)
//  {
//  /* USER CODE BEGIN USART3_MspInit 0 */
//
//  /* USER CODE END USART3_MspInit 0 */
//    /* Peripheral clock enable */
//    __HAL_RCC_USART3_CLK_ENABLE();
//
//    __HAL_RCC_GPIOD_CLK_ENABLE();
//    /**USART3 GPIO Configuration
//    PD8     ------> USART3_TX
//    PD9     ------> USART3_RX
//    */
//    GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN USART3_MspInit 1 */
//
//  /* USER CODE END USART3_MspInit 1 */
//  }
//
//}
//
//// Zj
//void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
//{
//
//  if(huart->Instance==UART4)
//  {
//  /* USER CODE BEGIN UART4_MspDeInit 0 */
//
//  /* USER CODE END UART4_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_UART4_CLK_DISABLE();
//
//    /**UART4 GPIO Configuration
//    PA0     ------> UART4_TX
//    PA1     ------> UART4_RX
//    */
//    HAL_GPIO_DeInit(GPIOA, ARD_D1_Pin|ARD_D0_Pin);
//
//  /* USER CODE BEGIN UART4_MspDeInit 1 */
//
//  /* USER CODE END UART4_MspDeInit 1 */
//  }
//  else if(huart->Instance==USART1)
//  {
//  /* USER CODE BEGIN USART1_MspDeInit 0 */
//
//  /* USER CODE END USART1_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART1_CLK_DISABLE();
//
//    /**USART1 GPIO Configuration
//    PB6     ------> USART1_TX
//    PB7     ------> USART1_RX
//    */
//    HAL_GPIO_DeInit(GPIOB, ST_LINK_UART1_TX_Pin|ST_LINK_UART1_RX_Pin);
//
//  /* USER CODE BEGIN USART1_MspDeInit 1 */
//
//  /* USER CODE END USART1_MspDeInit 1 */
//  }
//  else if(huart->Instance==USART3)
//  {
//  /* USER CODE BEGIN USART3_MspDeInit 0 */
//
//  /* USER CODE END USART3_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_USART3_CLK_DISABLE();
//
//    /**USART3 GPIO Configuration
//    PD8     ------> USART3_TX
//    PD9     ------> USART3_RX
//    */
//    HAL_GPIO_DeInit(GPIOD, INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin);
//
//  /* USER CODE BEGIN USART3_MspDeInit 1 */
//
//  /* USER CODE END USART3_MspDeInit 1 */
//  }
//
//}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
