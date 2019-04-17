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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
// #include "stm32l4xx_hal_gpio_ex.h"

#include "gps.h"
#include "gps_buff.h"

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

// Zj
// d1 and d0
UART_HandleTypeDef huart4;

// usb
UART_HandleTypeDef huart1;

uint8_t gps_tx_data[] = ""
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


uint8_t gps_rx_data[] = ""
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



uint8_t gps_rx;
typedef struct
{
	float latitude;
	float longitude;
} GPS_data;

GPS_data obj;
uint8_t temp_buf[500];


// iot original
static void Console_UART_Init(void);
static void RTC_Init(void);
static void Button_ISR(void);
static void cloud_test(void const *arg);

// Zj
static void MX_UART4_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void interruptInit(void);
void send_GPS_Uart4(void);
GPS_data buff_manage(uint8_t* buffer);
int count = 0;


// Zj
/*
void Uart4_RX_Int_Handler(void)
{
  // Un-comment for debugging
  __NOP();

  // HAL_UART_Transmit(&huart1, buffer_rx, sizeof(buffer_rx), UART_TIMEOUT_100_MS);
  if(count < 500)
  {
    temp_buf[count++] = gps_rx;
  }

  else
  {
    obj = buff_manage(temp_buf);
    count = 0; 					// testing purpose, need to be changed
  }

  // something here might need to be switched
  HAL_UART_Transmit(&huart1, &gps_rx, sizeof(gps_rx), 100);
  // HAL_UART_Transmit(&huart1, &obj.latitude, sizeof(obj.latitude), 100);
}
*/


UART_HandleTypeDef s_UARTHandle;

//#################################################################


/* GPS handle  */
gps_t hgps;

/* GPS buffer */
gps_buff_t hgps_buff;
uint8_t hgps_buff_data[12];


static size_t write_ptr;

static void
uart_irqhandler(void) {
    /* Make interrupt handler as fast as possible */
    /* Only write to received buffer and process later */
    if (write_ptr < strlen(temp_buf)) {
        /* Write to buffer only */
        gps_buff_write(&hgps_buff, &temp_buf[write_ptr++], 1); // copy gps_rx_data to hgps_buff....................
    }
}
//##################################################################

int main(void)
{
	HAL_Init();


	// Configure the system clock
	SystemClock_Config();

	// Luke UART

	// __USART4_CLK_ENABLE();
	// __UART4_CLK_ENABLE();
	// __GPIOA_CLK_ENABLE();




	Periph_Config();

	BSP_LED_Init(LED_GREEN);
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	// RNG init function
	hrng.Instance = RNG;
	if (HAL_RNG_Init(&hrng) != HAL_OK)
	{
		Error_Handler();
	}

	// RTC init
	RTC_Init();

	/* UART console init */
	Console_UART_Init();

	// Zj
	// interruptInit();
	// MX_UART4_Init();
	// MX_USART1_UART_Init();

	// *** UART4 INIT BEGIN ***
	// Zj
	__HAL_RCC_UART4_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Alternate = GPIO_AF8_UART4;
	// GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;

	// Zj
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIO_InitStructure.Pin = GPIO_PIN_1;
	// GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	// HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	s_UARTHandle.Instance        = UART4;
	s_UARTHandle.Init.BaudRate   = 9600;
	s_UARTHandle.Init.WordLength = UART_WORDLENGTH_8B;
	s_UARTHandle.Init.StopBits   = UART_STOPBITS_1;
	s_UARTHandle.Init.Parity     = UART_PARITY_NONE;
	s_UARTHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	s_UARTHandle.Init.Mode       = UART_MODE_TX_RX;

	// Zj
	s_UARTHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	s_UARTHandle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	s_UARTHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	// *** UART4 INIT END ***

	if (HAL_UART_Init(&s_UARTHandle) != HAL_OK) printf("bkpt 255\n");
	printf("Uart Initialized\n");

	#if (defined(__GNUC__) && !defined(__CC_ARM))

		/* Do not buffer stdout, so that single chars are output without any delay to the console. */
		setvbuf(stdout, NULL, _IONBF, 0);
	#endif

	// Configure Buttons Luke
	/*
	__GPIOD_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef myPin;
	myPin.Pin = GPIO_PIN_0;
	myPin.Mode = GPIO_MODE_INPUT;
	myPin.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &myPin);
	printf("Pin initialized\n");
	*/

	// This code works
	/*
	while (1)
	{
		if (GPIOA->IDR & GPIO_PIN_0)
		{
		  printf("Button Pressed\n");
		}
		HAL_Delay(1000);
	}
	*/

	printf("Entering Task Loop...\n");


	// Send data on UART4 to test loopback mode
	for(int i = 0; i <sizeof(temp_buf); i++)
	{
		uint8_t tx = gps_tx_data[i];

		//HAL_UART_Transmit(&s_UARTHandle, &tx, sizeof(tx), 100);
		HAL_UART_Receive(&s_UARTHandle, &temp_buf[i], sizeof(tx), 100);

		printf("%c", temp_buf[i]);

	}


//########################################################parser from internet#############################



    uint8_t rx;

    gps_init(&hgps);                            /* Init GPS */

    /* Create buffer for received data */
    gps_buff_init(&hgps_buff, hgps_buff_data, sizeof(hgps_buff_data));

    while (1) {
        /* Add new character to buffer */
        /* Fake UART interrupt handler on host microcontroller */
        uart_irqhandler(); // copy the gps_rx_data to buffer

        /* Process all input data */
        /* Read from buffer byte-by-byte and call processing function */
        if (gps_buff_get_full(&hgps_buff)) {    /* Check if anything in buffer now */
            while (gps_buff_read(&hgps_buff, &rx, 1)) {
                gps_process(&hgps, &rx, 1);     /* Process byte-by-byte */
            }
        } else {

            printf("Latitude: %f degrees\r\n", hgps.latitude);
            printf("Longitude: %f degrees\r\n", hgps.longitude);
            printf("Altitude: %f meters\r\n", hgps.altitude);
            printf("Hours: %d \r\n", hgps.hours);
            printf("Minute: %d\r\n", hgps.minutes);
            printf("Seconds: %d seconds\r\n", hgps.seconds);
            printf("Satelites in Use: %d\r\n", hgps.sats_in_use);
            printf("Satelites in View: %d\r\n", hgps.sats_in_view);
            printf("-----------------------------------\r\n");
            printf("-----------------------------------\r\n");
            printf("-----------------------------------\r\n");
            //printf("%ld\n", strlen(gps_rx_data));
            break;
        }
    }










//#######################################################################################################



	/*
	// Loop for testing
	while (1)
	{
		// HAL_UART_Transmit(&s_UARTHandle, &buffer, sizeof(buffer), 100);
		// HAL_UART_Receive(&s_UARTHandle, &buffer2, sizeof(buffer), 100);
		// HAL_Delay(1000);
		// printf("%c\n", buffer2);
		// HAL_Delay(1000);
		// send_GPS_Uart4();
	}
	*/

	// cloud_test(0);
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

	// Zj
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	// same
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;

	// diff: iot = 11, zj = 6
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;

	// same
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;

	// diff: iot = 6 & 20, zj = 1 & 40
	RCC_OscInitStruct.PLL.PLLM = 6;
	RCC_OscInitStruct.PLL.PLLN = 20;

	// same
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	clocks dividers */
	// same
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}

	// Zj diff
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;		// RCC_PERIPHCLK_USART1 |
	// PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	// Configure the main internal regulator output voltage
	// Zj
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/* Enable MSI PLL mode */
	// in iot, not Zj
	HAL_RCCEx_EnableMSIPLLMode();
}

// Zj
GPS_data buff_manage(uint8_t* buffer)
{
	int index = 0;
	int $count = 0;
	int comma_count = 0;
	char latitude[6] = {0}; 	//assuming the latitude is only 6 in size
	int alt_index = 0;
	int alt_flag = 0; 			// N for pas S for neg
	char longitude[6] = {0};	// assuming the longitude is only 6 in size
	int long_index = 0;
	int long_flag = 0; 			// E for pas W for neg
	GPS_data data; 				// empty struct
	while ($count != 3)
	{
		if (temp_buf[index] == '$')
		{
			$count++;
		}
		index++;
	} // after the while, the next char is 'G' for GPGGA

	while (comma_count != 2)
	{
		if (temp_buf[index] == ',')
		{
			comma_count++;
		}
		index++;
	} // checking for comma, it points to the first char of latitude after the while is finished

	while (temp_buf[index] != '.')
	{
		latitude[alt_index++] = temp_buf[index++];
	} // finish loading the digits before . for latitude
	comma_count = 0;
	while (comma_count != 1)
	{
		if (temp_buf[index] == ',')
		{
			comma_count++;
		}
		index++;
	} // skip the comma, it points to 'N' right now

	comma_count = 0;
	if (temp_buf[index] == 'N')
	{
		alt_flag = 1;
		index++; // skip the ',' and points to the first digit on the longitude
	}

	index++; // get over the comma
	while (temp_buf[index] != '.')
	{
		longitude[long_index++] = temp_buf[index++];
	} // finish loading the digits before . for longitude


	while (comma_count != 1)
	{
		if (temp_buf[index] == ',')
		{
			comma_count++;
		}
		index++;
	} // skip the comma, it points to 'W' right now

	if (temp_buf[index] == 'E')
	{
		long_flag = 1;
		index++;
	} // finish the loading alt and lang, now converting char to float

	alt_index = sizeof(latitude)-1; // going voer from 0
	data.latitude = 0; // init the latitude
	int tp;
	while (latitude[alt_index] == NULL)
	{
		alt_index--;
	}

	tp = alt_index; // keep alt_index
	float div = 1;
	while ((alt_index-2)!= 0)
	{
		div = div * 10;
		alt_index--;
	}

	alt_index = tp;
	float temp_int = 0;
	for (int i = 0; i <= alt_index; i++)
	{
		temp_int = latitude[i] - '0';
		data.latitude = data.latitude + temp_int * div;
		div = div / 10;
	}
	if(alt_flag == 0)
	{
		data.latitude = data.latitude*-1;
	}

	// data.latitude
	long_index = sizeof(longitude) - 1; // going voer from 0
	data.longitude = 0; // init the latitude
	while (longitude[long_index] == NULL)
	{
		long_index--;
	}
	tp = long_index;
	div = 1;

	while ((long_index-2) != 0)
	{
		div = div * 10;
		long_index--;
	}

	long_index =tp;
	temp_int = 0;
	for (int i = 0; i <= long_index; i++)
	{
		temp_int = longitude[i] - '0';
		data.longitude = data.longitude + temp_int * div;
		div = div / 10;
	}

	if(long_flag == 0)
	{
		data.longitude = data.longitude*-1;
	}

	return data;
}

// Zj
void send_GPS_Uart4(void)
{
	// Enable RX interrupt for Uart4
	HAL_UART_Receive_IT(&huart4, &gps_rx, 1);

	// Reset elements of RX
	// memset(buffer_rx, 0, sizeof(buffer_rx));
	static int i;

	// Send data on UART4 to test loopback mode
	for(i = 0; i < sizeof(gps_tx_data); i++)
	{
		uint8_t tx = gps_tx_data[i];
		HAL_UART_Transmit(&huart4, &tx, 1, 100);
		HAL_UART_Receive_IT(&huart4, &gps_rx, 1);
	}
	HAL_Delay(1000);
}

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

static void MX_UART4_Init(void)
{
	printf("Initializing UART4...\n");
	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 9600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		printf("Error initializing UART4\n");
		Error_Handler();
	}
	else printf("UART4 initialized\n");

	/* USER CODE BEGIN UART4_Init 2 */
	/* USER CODE END UART4_Init 2 */
}

static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
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
// IoT
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

#if (defined(__GNUC__) && !defined(__CC_ARM))
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

// Luke
static void interruptInit(void)
{
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  // I don't think we need all that other GPIO setup

  /* EXTI interrupt init*/
  // Zj
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


/**
  * @brief RTC init function
  */
static void RTC_Init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  /* 32.768kHz LSE clock input */
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

// IoT
static void cloud_test(void const *arg)
{
	platform_init();
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
    case (USER_BUTTON_PIN):
    {
      Button_ISR();
      break;
    }
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    default:
    {
      break;
    }
  }
}


void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

void Error_Handler(void)
{
	printf("An error has occurred\n");
	/*
	while(1)
	{
		BSP_LED_Toggle(LED_GREEN);
		HAL_Delay(200);
	}
	*/
}

// Zj
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// testUart4_Int();

	// uncomment for interrrupts
	// Uart4_RX_Int_Handler();
}

