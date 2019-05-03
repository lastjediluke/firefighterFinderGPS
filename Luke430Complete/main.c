/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "googleiot.h"

/* Global variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
RNG_HandleTypeDef hrng;
net_hnd_t         hnet; /* Is initialized by cloud_main(). */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef console_uart;
static volatile uint8_t button_flags = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
void SPI_WIFI_ISR(void);
static void Console_UART_Init(void);
static void RTC_Init(void);
static void Button_ISR(void);
static void cloud_test(void const *arg);

/* Private functions ---------------------------------------------------------*/
uint8_t buffer_rx[4];
uint8_t gps_tx_data2[] = ""
"$GPRMC,183729,A,3907.356,N,12102.482,W,000.0,360.0,080301,015.5,E*6F\r\n"
"$GPRMB,A,,,,,,,,,,,,V*71\r\n"
"$GPGGA,183730,3907.356,N,12102.482,W,1,05,1.6,646.4,M,-24.1,M,,*75\r\n"
"$GPGSA,A,3,02,,,07,,09,24,26,,,,,1.6,1.6,1.0*3D\r\n"
"$GPGSV,2,1,08,02,43,088,38,04,42,145,00,05,11,291,00,07,60,043,35*71\r\n"
"$GPGSV,2,2,08,08,02,145,00,09,46,303,47,24,16,178,32,26,18,231,43*77\r\n"
"$PGRME,22.0,M,52.9,M,51.0,M*14\r\n"
"$GPGLL,3907.360,N,12102.481,W,183730,A*33\r\n"
"$PGRMZ,2062,f,3*2D\r\n"
"$PGRMM,WGS84*06\r\n"
"$GPBOD,,T,,M,,*47\r\n"
"$GPRTE,1,1,c,0*07\r\n"
"$GPRMC,183731,A,3907.482,N,12102.436,W,000.0,360.0,080301,015.5,E*67\r\n"
"$GPRMB,A,,,,,,,,,,,,V*71\r\n";

/*
uint8_t gps_tx_data[] = ""
"$GPGGA,183730,3907.356,N,12102.482,W,1,05,1.6,646.4,M,-24.1,M,,*75\r\n"
"$GPGSA,A,3,02,,,07,,09,24,26,,,,,1.6,1.6,1.0*3D\r\n";
*/

uint8_t luke_gps[] = ""
"$NGGA,014626.00,3720.19969,N,12152.86258,W,2,12,0.67,36.3,M,-29.9,M,,0000*43";

// uint8_t gps_rx;

// GPS handle
// gps_t hgps;

// GPS buffer
// gps_buff_t hgps_buff;
// uint8_t hgps_buff_data[12];

void print_GPS_values(void)
{

	//__NOP();
    //HAL_UART_Transmit(&huart4, (uint8_t *)&gh->sats_in_use, sizeof(gh->sats_in_use), 10);
    /*HAL_UART_Transmit(&huart4, (uint8_t *)&GPS_Parsed_Flag, sizeof(GPS_Parsed_Flag), 10);
    GPS_Parsed_Flag = 0;*/
    //HAL_UART_Transmit(&huart4, (uint8_t *)"Saumil\n", sizeof("Saumil\n"), 10);
}

/**
  * @brief  Main program
  * @param  None
  * @retval Status
  */
int main(void)
{
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  Periph_Config();

  BSP_LED_Init(LED_GREEN);
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* RNG init function */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }

  // RTC init
  RTC_Init();

  // UART console init
  Console_UART_Init();
  printf("Console Initialized...\n");

  // Configure Buttons Luke

     __GPIOC_CLK_ENABLE();
  	// __GPIOA_CLK_ENABLE();
  	GPIO_InitTypeDef myPin;
  	myPin.Pin = GPIO_PIN_2;
  	// myPin.Mode = GPIO_MODE_INPUT;	// not interrupt
  	myPin.Pull = GPIO_NOPULL;
  	myPin.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  	myPin.Mode = GPIO_MODE_IT_RISING;
  	HAL_GPIO_Init(GPIOC, &myPin);

  	GPIO_InitTypeDef myPin3;
  	myPin3.Pin = GPIO_PIN_3;
	// myPin.Mode = GPIO_MODE_INPUT;	// not interrupt
	myPin3.Pull = GPIO_NOPULL;
	myPin3.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	myPin3.Mode = GPIO_MODE_IT_RISING;
	HAL_GPIO_Init(GPIOC, &myPin3);


  	HAL_NVIC_SetPriority(EXTI2_IRQn, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0x0F, 0x00);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  	printf("Pin initialized\n");


  	// This code works


  	// while(1)
  	// {
  		/*
  		if (GPIOC->IDR & GPIO_PIN_14)
  		{
  		  printf("Button Pressed\n");
  		}
  		HAL_Delay(1000);
  		*/
  	// }



  uartAndGpsInit();

// #ifdef FIREWALL_MBEDLIB
// firewall_init();
// #endif

  printf("Starting Cloud Test...\n");
  cloud_test(0);
  printf("End main\n");
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI MSE)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 48000000
  *            PLL_M                          = 6
  *            PLL_N                          = 20
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable MSI PLL mode */
  HAL_RCCEx_EnableMSIPLLMode();
}


/**
  * @brief Set LED state
  */
void Led_SetState(bool on)
{
  if (on == true)
  {
    BSP_LED_On(LED_GREEN);
  }
  else
  {
    BSP_LED_Off(LED_GREEN);
  }
}


/**
 * @brief Blink LED for 'count' cycles of 'period' period and 'duty' ON duration.
 * duty < 0 tells to start with an OFF state.
 */
void Led_Blink(int period, int duty, int count)
{
  if ( (duty > 0) && (period >= duty) )
  {
    /*  Shape:   ____
                  on |_off__ */
    do
    {
      Led_SetState(true);
      HAL_Delay(duty);
      Led_SetState(false);
      HAL_Delay(period - duty);
    } while (count--);
  }
  if ( (duty < 0) && (period >= -duty) )
  {
    /*  Shape:         ____
                __off_| on   */
    do
    {
      Led_SetState(false);
      HAL_Delay(period + duty);
      Led_SetState(true);
      HAL_Delay(-duty);
    } while (count--);
  }
}

/**
  * @brief Update button ISR status
  */
static void Button_ISR(void)
{
  button_flags++;
}




/**
  * @brief Waiting for button to be pushed
  */
uint8_t Button_WaitForPush(uint32_t delay)
{
  uint32_t time_out = HAL_GetTick()+delay;
  do
  {
    if (button_flags > 1)
    {
      button_flags = 0;
      return BP_MULTIPLE_PUSH;
    }

    if (button_flags == 1)
    {
      button_flags = 0;
      return BP_SINGLE_PUSH;
    }
  }
  while( HAL_GetTick() < time_out);
  return BP_NOT_PUSHED;
}

/**
  * @brief Waiting for button to be pushed
  */
uint8_t Button_WaitForMultiPush(uint32_t delay)
{
  HAL_Delay(delay);
  if (button_flags > 1)
  {
    button_flags = 0;
    return BP_MULTIPLE_PUSH;
  }

  if (button_flags == 1)
  {
    button_flags = 0;
    return BP_SINGLE_PUSH;
  }
  return BP_NOT_PUSHED;
}


/**
  * @brief UART console init function
  */
static void Console_UART_Init(void)
{
  console_uart.Instance = USART1;
  console_uart.Init.BaudRate = 115200;
  console_uart.Init.WordLength = UART_WORDLENGTH_8B;
  console_uart.Init.StopBits = UART_STOPBITS_1;
  console_uart.Init.Parity = UART_PARITY_NONE;
  console_uart.Init.Mode = UART_MODE_TX_RX;
  console_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  console_uart.Init.OverSampling = UART_OVERSAMPLING_16;
#ifdef UART_ONE_BIT_SAMPLE_DISABLE
  console_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  console_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif
  BSP_COM_Init(COM1,&console_uart);
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */


#if (defined(__GNUC__) && !defined(__CC_ARM))
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  while (HAL_OK != HAL_UART_Transmit(&console_uart, (uint8_t *) &ch, 1, 30000))
  {
    ;
  }
  return ch;
}

/**
  * @brief  Retargets the C library scanf function to the USART.
  * @param  None
  * @retval None
  */
GETCHAR_PROTOTYPE
{
  /* Place your implementation of fgetc here */
  /* e.g. read a character on USART and loop until the end of read */
  uint8_t ch = 0;
  while (HAL_OK != HAL_UART_Receive(&console_uart, (uint8_t *)&ch, 1, 30000))
  {
    ;
  }
  return ch;
}

/**
  * @brief RTC init function
  */
static void RTC_Init(void)
{
    /**Initialize RTC Only */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
#ifdef RTC_OUTPUT_REMAP_NONE
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
#endif
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
}

static void cloud_test(void const *arg)
{
  platform_init();
  // subscribe_publish_sensor_values();		// Saumil
  // uartAndGpsInit();
  gcp_main(0);
  platform_deinit();
}


/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_13):
    {
      Button_ISR();
      break;
    }

	case (GPIO_PIN_1):
	{
		SPI_WIFI_ISR();
		break;
	}

	case (GPIO_PIN_2):
	{
	  floorButtonISR();
	  break;
	}

	case (GPIO_PIN_3):
	{
	  dangerButtonISR();
	  break;
	}

	case (GPIO_PIN_4):
	{
	  Button_ISR();
	  break;
	}

    default:
    {
      break;
    }
  }
}

// uart rcv callback

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  while(1)
  {
    BSP_LED_Toggle(LED_GREEN);
    HAL_Delay(200);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
