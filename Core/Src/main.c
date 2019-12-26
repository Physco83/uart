/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define UL (unsigned long *)
#define UI (unsigned int  *)
#define STACK_TOP    0x20000800
#define RCC__BASE     0x40021000
#define GPIOB_        0x48000400
#define GPIO_MODER   0x00000000
#define ABH_OFFSET   0x00000014
#define MODER_ENABLE 0x00000040
#define MODER_RESET  ~0XF0
#define PORTB_CL_EN  1<<18     //0x00040000
#define GPIO_OUTPUT  0x00000014
#define LED3_BIT_ON  1<<3      //0x00000008

#define TIM2_         0x40000000
#define TIM3_         0x40000400
#define NVIC_ISER    0xE000E100
#define NVIC_INTER_EN 1<<16
#define TIM3_STAT_R  0x00000010
#define TIM3_ST_R_RS 0x00000000
#define TIM3_COUNTER 0x00000024 //TIM3 counter offset, ONLY FIRST 16 BIT

#define APB1ENR      0x0000001C
#define TIM2_CL_EN   0x00000001
#define TIM3_CL_EN   1<<1      //0x00000002
#define TIM3_DIER    0x0000000C
#define TIM3_UIE     0x00000001
#define TIM3_PSC     0x00000028
#define TIM3_ARR     0x0000002C
#define TIM3_CR1     0x00000000
#define TIM3_COUNTER_EN 0x00001

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t tx_buff[] = "hello\r\n";


void dump_handler() {} // empty function

/*
void TIM3_IRK_handler() {

	*(UI (GPIOB_+GPIO_OUTPUT)) |= LED3_BIT_ON;
	if (*(UI (TIM3_ + TIM3_ARR)) == 10000) {
		return ;
	}
	if ((*(UI (GPIOB+GPIO_OUTPUT)) & LED3_BIT_ON) != 0) {//LED3 was enabled
        *(UI (GPIOB_+GPIO_OUTPUT)) &= ~LED3_BIT_ON;
        *(UI (TIM3_ + TIM3_ARR)) = 300;  //may be you won't see how it blinks/
    }  else {
        *(UI (GPIOB_+GPIO_OUTPUT)) |= LED3_BIT_ON;
        *(UI (TIM3_ + TIM3_ARR)) = 10000; //so now you have a lot of time to participate
    }
    *(UI (TIM3_+TIM3_STAT_R)) = TIM3_ST_R_RS;
}
*/

void waste_time () {
    for (int i  = 0; i<3; i++) {}
}


void led_initialization() {
    *(UI (RCC__BASE+ABH_OFFSET)) |= PORTB_CL_EN;// I/O port B clock enable
    waste_time();

    *(UI (GPIOB_+GPIO_MODER)) &= MODER_RESET; //reset moder GPIOB
    *(UI (GPIOB_+GPIO_MODER)) |= MODER_ENABLE;//general output purpose mode for GPIOB
}


void random_blink() {
	*(UI (TIM3+TIM3_STAT_R)) = TIM3_ST_R_RS;
	*(UI (TIM3+TIM3_COUNTER)) = 0x0;
	*(UI (TIM3_ + TIM3_PSC)) = 16000;
	*(UI (TIM3_ + TIM3_ARR)) = 4000 + (*(UI (TIM2_ + TIM3_COUNTER))) % 6000; //here you should make randomizatoR; was 8100
	waste_time();
	*(UI (TIM3_ + TIM3_CR1)) |= TIM3_COUNTER_EN; //counter enable

}

int myatoi(uint8_t *str, int am) {
	int result = 0;;
	int i = 0;
	while ((i < am) && (str[i] != '\0')) {
		result= result*10 + str[i] - '0';
		i++;
	}
	return result;
}

void myitoa(unsigned long num, uint8_t* str, int* n) {
	unsigned long copy = num;
	int c=0;
	while (copy>0) {
		c++;
		copy=copy/10;
	}
	*n = c+2;
	str[c] = '\r';
	str[c+1] = '\n';
	for (int i = 0; i < c; i++) {
		str[c-i-1] = num%10 + '0';
		num = num / 10;
	}
	return ;
}

void hi () {
	uint32_t HAL_DELAY = 100000;
	HAL_UART_Transmit(&huart1, tx_buff, 7, HAL_DELAY);
	tx_buff[0] = 'E';
	tx_buff[1] = 'n';
	tx_buff[2] = 't';
	tx_buff[3] = 'e';
	tx_buff[4] = 'r';
	tx_buff[5] = ' ';
	tx_buff[6] = 'p';
	HAL_UART_Transmit(&huart1, tx_buff, 7, HAL_DELAY);
	tx_buff[0] = 'l';
	tx_buff[1] = 'a';
	tx_buff[2] = 'y';
	tx_buff[3] = 'e';
	tx_buff[4] = 'r';
	tx_buff[5] = ' ';
	tx_buff[6] = 'n';
	HAL_UART_Transmit(&huart1, tx_buff, 7, HAL_DELAY);
	tx_buff[0] = 'u';
	tx_buff[1] = 'm';
	tx_buff[2] = 'b';
	tx_buff[3] = 'e';
	tx_buff[4] = 'r';
	tx_buff[5] = ' ';
	HAL_UART_Transmit(&huart1, tx_buff, 6, HAL_DELAY);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  led_initialization();



  uint32_t HAL_DELAY = 100000;
  hi();
  uint8_t rx_buff[1];
  HAL_UART_Receive(&huart1, rx_buff, 1, HAL_DELAY);
  tx_buff[0] = rx_buff[0];
  tx_buff[1] = '\r';
  tx_buff[2] = '\n';
  HAL_UART_Transmit(&huart1, tx_buff, 3, HAL_DELAY);
  int people_count = myatoi(rx_buff, 1);
  *(UI (GPIOB_+GPIO_OUTPUT)) &= ~LED3_BIT_ON;
  waste_time();

  unsigned long time_mas[people_count + 1];
  time_mas[0] = 0xFFFFFF;
  unsigned int best = 0;
  uint8_t tx_buff3[] = "player1\r\n";
  uint8_t time_buff[32];
  random_blink();
  *(UI (GPIOB_+GPIO_OUTPUT)) &= ~LED3_BIT_ON;
  for (uint8_t i = 0; i < people_count; i++) {
	  HAL_UART_Transmit(&huart1, tx_buff3, 10, HAL_DELAY);
	  random_blink();
	  HAL_UART_Receive(&huart1, rx_buff, 1, HAL_DELAY);
	  *(UI (GPIOB_+GPIO_OUTPUT)) &= ~LED3_BIT_ON;
	  time_mas[i+1] = *(UI (TIM3_ + TIM3_COUNTER));
	  if (time_mas[i + 1] < time_mas[best]) {
		  best = i + 1;
	  }
	  int size;
	  myitoa(time_mas[i+1], time_buff, &size);
	  HAL_UART_Transmit(&huart1, time_buff, size, HAL_DELAY);
	  tx_buff3[6] += 1;

  }
  uint8_t tx_buff2[] = "result is";
  HAL_UART_Transmit(&huart1, tx_buff2, 9, HAL_DELAY);
  tx_buff2[0] = 32;
  tx_buff2[1] = best + 48;
  waste_time();
  HAL_UART_Transmit(&huart1, tx_buff2, 2, HAL_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //HAL_UART_Transmit(&huart1, tx_buff, 7, HAL_DELAY);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  *(UI (TIM3_ + 0x8)) = 0x0; //don't erase this line!
  *(UI (NVIC_ISER)) |= NVIC_INTER_EN; //tim3 handler has't been default since now; interrupt mask register??
  *(UI (TIM3_ + TIM3_DIER)) |= TIM3_UIE; //Update interrupt enabled
  *(UI (TIM3_ + TIM3_PSC)) = 8000;
  *(UI (TIM3_ + TIM3_ARR)) = 1025;
  *(UI (TIM3_ + TIM3_CR1)) |= 1<<3; //OPM enabled
  waste_time();



  *(UI (RCC_BASE+APB1ENR)) |= TIM2_CL_EN;//TIM2 clock enabled
  waste_time();

  *(UI (TIM2_ + TIM3_DIER)) |= TIM3_UIE; //Update interrupt enabled
  *(UI (TIM2_ + TIM3_PSC)) = 100;
  *(UI (TIM2_ + TIM3_ARR)) = 1000025;
  waste_time();
  *(UI (TIM2_ + TIM3_CR1)) |= TIM3_COUNTER_EN; //counter enable
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
