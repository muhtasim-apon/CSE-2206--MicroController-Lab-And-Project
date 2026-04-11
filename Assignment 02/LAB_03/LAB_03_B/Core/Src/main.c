/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char str0[120] = "[#] | Block Description                       |  Cycles  |    ns    |   us   |  ms  |  s  |  Baud  \r\n";
char str1[120] = "[1] | Bubble Sort (n = 100) in DWT            |          |          |        |      |     | -\r\n";
char str2[120] = "    | Bubble Sort (n = 100) in TIM            | -        |          |        |      |     | -\r\n";
char str5[120] = "[3] | USART2_SendString(48) in DWT            |          |          |        |      |     |        \r\n";
char str6[120] = "    | USART2_SendString(48) in TIM            | -        |          |        |      |     |        \r\n";
char str7[120] = "[4] | Integer sq.rt. via Binary Search in DWT |          |          |        |      |     | -\r\n";
char str8[120] = "    | Integer sq.rt. via Binary Search in TIM | -        |          |        |      |     | -\r\n";
char str9[120] = "[5] | Byte-by-Byte Memory Copy in DWT         |          |          |        |      |     | -\r\n";
char str10[120]= "    | Byte-by-Byte Memory Copy in TIM         |          |          |        |      |     | -\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void DWT_CONFIG(void);
uint32_t DWT_TIME(uint32_t cycles, uint32_t factor);
uint32_t TIM_TIME(uint32_t micros, uint32_t factor);
void CONVERTER(uint32_t x, char *str);
void CALC_DWT(char* str, uint32_t cycles);
void CALC_TIM(char* str, uint32_t micros);
void SWAP(int *a, int *b);
void BUBBLE_SORT(int arr[]);
int SQRT_BINARY(int i);
void PROFILE_01_DWT(void);
void PROFILE_01_TIM(void);
void PROFILE_03_DWT(void);
void PROFILE_03_TIM(void);
void PROFILE_04_DWT(void);
void PROFILE_04_TIM(void);
void PROFILE_05_DWT(void);
void PROFILE_05_TIM(void);
void TABLE_OUT(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DWT_CONFIG(void)
{
	CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enabling the Trace Enable Control
	DWT -> CYCCNT = 0; // Resetting Cycle Counter
	DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enabling the CYCCNTENA bit for starting count
}

uint32_t DWT_TIME(uint32_t cycles, uint32_t factor)
{
    double seconds = cycles / 90000000.0; // As SYSCLK = 90 MHz
    seconds *= factor;
    return (uint32_t)seconds;
}

uint32_t TIM_TIME(uint32_t micros, uint32_t factor)
{
	double seconds = micros / 1000000.0;
	seconds *= factor;
	return (uint32_t)seconds;
}

void CONVERTER(uint32_t x, char *str)
{
    char buf[11];
    int i = 0;
    if (x == 0)
    {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    while (x > 0)
    {
        buf[i++] = (char)('0' + (x % 10));
        x /= 10;
    }
    for (int j = 0; j < i; j++) str[j] = buf[i - (j + 1)];
    str[i] = '\0';
}

void CALC_DWT(char* str, uint32_t cycles)
{
	char temp[16];

	CONVERTER(cycles, temp);
	memcpy(&str[48], temp, strlen(temp));

	uint32_t t_nanos = DWT_TIME(cycles, 1e9);
	CONVERTER(t_nanos, temp);
	memcpy(&str[59], temp, strlen(temp));

	uint32_t t_micros = DWT_TIME(cycles, 1e6);
	CONVERTER(t_micros, temp);
	memcpy(&str[70], temp, strlen(temp));

	uint32_t t_millis = DWT_TIME(cycles, 1e3);
	CONVERTER(t_millis, temp);
	memcpy(&str[79], temp, strlen(temp));

	uint32_t t_secs = DWT_TIME(cycles, 1);
	CONVERTER(t_secs, temp);
	memcpy(&str[86], temp, strlen(temp));
}

void CALC_TIM(char* str, uint32_t micros)
{
	char temp[16];

	uint32_t t_nanos = TIM_TIME(micros, 1e9);
	CONVERTER(t_nanos, temp);
	memcpy(&str[59], temp, strlen(temp));

	uint32_t t_micros = TIM_TIME(micros, 1e6);
	CONVERTER(t_micros, temp);
	memcpy(&str[70], temp, strlen(temp));

	uint32_t t_millis = TIM_TIME(micros, 1e3);
	CONVERTER(t_millis, temp);
	memcpy(&str[79], temp, strlen(temp));

	uint32_t t_secs = TIM_TIME(micros, 1);
	CONVERTER(t_secs, temp);
	memcpy(&str[86], temp, strlen(temp));
}

void SWAP(int *a, int *b)
{
    int tmp = *a;
    *a = *b;
    *b = tmp;
}

void BUBBLE_SORT(int arr[])
{
	int n = 100;
	while (n) arr[100 - n] = n--; // Array Buildup
	for (int i = 0; i<100; i++)
		for (int j = 0; j < 100-(i+1); j++)
			if (arr[j] > arr[j+1]) SWAP(&arr[j], &arr[j+1]);
}

int SQRT_BINARY(int i)
{
	int left = 0, right = i, mid, root = 0;
	while (left <= right)
	{
		mid = left + (right - left)/2;
		if ((long long) mid * mid <= i)
		{
			root = mid;
			left = mid + 1;
		}
		else right = mid - 1;
	}
	return root;
}

void PROFILE_01_DWT(void)
{
	int arr[100] = {0};
	uint32_t T1 = DWT -> CYCCNT;
	BUBBLE_SORT(arr);
	uint32_t T2 = DWT -> CYCCNT;
	uint32_t cycles = T2 - T1;
	CALC_DWT(str1, cycles);
}

void PROFILE_01_TIM(void)
{
	int arr[100] = {0};
	TIM2 -> CNT = 0;
	uint32_t T1 = __HAL_TIM_GET_COUNTER(&htim2);
	BUBBLE_SORT(arr);
	uint32_t T2 = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t micros = T2 - T1;
	CALC_TIM(str2, micros);
}

void PROFILE_03_DWT(void)
{
	char* c = "Profiling code speed at 90MHz clock ticks.....\r\n";
	uint32_t T1 = DWT -> CYCCNT;
	HAL_UART_Transmit(&huart2, (uint8_t*)c, strlen(c), HAL_MAX_DELAY);
	uint32_t T2 = DWT -> CYCCNT;
	uint32_t cycles = T2 - T1;
	char t[10];
	CALC_DWT(str5, cycles);
	double micros = cycles / 90.0;
	micros = 480 / micros; // 8 bit sending, 1 start bit, 1/2 stop bits, generally 10 bits
	micros *= 1e6;
	CONVERTER((uint32_t)micros, t);
	memcpy(&str5[92], t, strlen(t));
}

void PROFILE_03_TIM(void)
{
	char* c = "Profiling code speed at 90MHz clock ticks.....\r\n";
	TIM2 -> CNT = 0;
	uint32_t T1 = __HAL_TIM_GET_COUNTER(&htim2);
	HAL_UART_Transmit(&huart2, (uint8_t*)c, strlen(c), HAL_MAX_DELAY);
	uint32_t T2 = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t micros = T2 - T1;
	char t[10];
	CALC_TIM(str6, micros);
	double micro = 480.0 / micros; // 8 bit sending, 1 start bit, 1/2 stop bits, generally 10 bits
	micro *= 1e6;
	CONVERTER((uint32_t)micro, t);
	memcpy(&str6[92], t, strlen(t));
}

void PROFILE_04_DWT(void)
{
	uint32_t T1 = DWT -> CYCCNT;
	for (int i = 1; i <= 1000; i++) SQRT_BINARY(i);
	uint32_t T2 = DWT -> CYCCNT;
	uint32_t cycles = T2 - T1;
	CALC_DWT(str7, cycles);
}

void PROFILE_04_TIM(void)
{
	TIM2 -> CNT = 0;
	uint32_t T1 = __HAL_TIM_GET_COUNTER(&htim2);
	for (int i = 1; i <= 1000; i++) SQRT_BINARY(i);
	uint32_t T2 = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t micros = T2 - T1;
	CALC_TIM(str8, micros);
}

void PROFILE_05_DWT(void)
{
	uint8_t src[512];
	uint8_t dest[512];
	for (int i = 0; i < 512; i++) src[i] = (uint8_t) (i & 0xF);
	uint32_t T1 = DWT -> CYCCNT;
	for (int i = 0; i < 512; i++) dest[i] = src[i];
	uint32_t T2 = DWT -> CYCCNT;
	uint32_t cycles = T2 - T1;
	CALC_DWT(str9, cycles);
}

void PROFILE_05_TIM(void)
{
	uint8_t src[512];
	uint8_t dest[512];
	for (int i = 0; i < 512; i++) src[i] = (uint8_t) (i & 0xF);
	TIM2 -> CNT = 0;
	uint32_t T1 = __HAL_TIM_GET_COUNTER(&htim2);
	for (int i = 0; i < 512; i++) dest[i] = src[i];
	uint32_t T2 = __HAL_TIM_GET_COUNTER(&htim2);
	uint32_t micros = T2 - T1;
	CALC_TIM(str10, micros);
}

void TABLE_OUT(void)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)str0, strlen(str0), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str1, strlen(str1), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str2, strlen(str2), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str5, strlen(str5), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str6, strlen(str6), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str7, strlen(str7), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str8, strlen(str8), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str9, strlen(str9), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)str10, strlen(str10), HAL_MAX_DELAY);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2); // Explicit counter start
  char* STRING = "LAB 03 (HAL): Duration Measurement with Code Profiling using USART\r\n\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)STRING, strlen(STRING), HAL_MAX_DELAY);

  char* c1 = "Task 01: Bubble Sort (Worst Case) for n = 100\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)c1, strlen(c1), HAL_MAX_DELAY);
  PROFILE_01_DWT();
  PROFILE_01_TIM();

  char* c3 = "Task 03: USART2 48-Byte Transmission Check\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)c3, strlen(c3), HAL_MAX_DELAY);
  PROFILE_03_DWT();
  PROFILE_03_TIM();

  char* c4 = "Task 04: Integer Square Root check by Binary Search for n = 1000\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)c4, strlen(c4), HAL_MAX_DELAY);
  PROFILE_04_DWT();
  PROFILE_04_TIM();

  char* c5 = "Task 05: Memory Copy of 512 Bytes byte-by-byte\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)c5, strlen(c5), HAL_MAX_DELAY);
  PROFILE_05_DWT();
  PROFILE_05_TIM();

  char* f = "\nFinal Calculation\r\n\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)f, strlen(f), HAL_MAX_DELAY);
  TABLE_OUT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
