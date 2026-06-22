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
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP280_REG_CHIP_ID     0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_START 0x88

#define CHIP_ID_BMP280  0x58
#define CHIP_ID_BME280  0x60

#define CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint16_t T1;
static  int16_t T2, T3;
static uint16_t P1;
static  int16_t P2, P3, P4, P5;
static  int16_t P6, P7, P8, P9;
static  int32_t t_preq;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t SPI_Transfer(uint8_t data)
{
    uint8_t rx = 0;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static uint8_t SPI_ReadByte(uint8_t reg)
{
    CS_LOW();
    SPI_Transfer(reg | 0x80);
    uint8_t val = SPI_Transfer(0xFF);
    CS_HIGH();
    return val;
}

static void SPI_WriteByte(uint8_t reg, uint8_t data)
{
    CS_LOW();
    SPI_Transfer(reg & 0x7F);
    SPI_Transfer(data);
    CS_HIGH();
}

static void SPI_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    CS_LOW();
    SPI_Transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI_Transfer(0xFF);
    CS_HIGH();
}

static void Load_Calibration(void)
{
    uint8_t c[24];
    SPI_ReadBurst(BMP280_REG_CALIB_START, c, 24);
    T1 = (uint16_t)(c[1]  << 8 | c[0]);
    T2 =  (int16_t)(c[3]  << 8 | c[2]);
    T3 =  (int16_t)(c[5]  << 8 | c[4]);
    P1 = (uint16_t)(c[7]  << 8 | c[6]);
    P2 =  (int16_t)(c[9]  << 8 | c[8]);
    P3 =  (int16_t)(c[11] << 8 | c[10]);
    P4 =  (int16_t)(c[13] << 8 | c[12]);
    P5 =  (int16_t)(c[15] << 8 | c[14]);
    P6 =  (int16_t)(c[17] << 8 | c[16]);
    P7 =  (int16_t)(c[19] << 8 | c[18]);
    P8 =  (int16_t)(c[21] << 8 | c[20]);
    P9 =  (int16_t)(c[23] << 8 | c[22]);
}

static int32_t Compensate_Temperature(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)T1)) *
              ((adc_T >> 4) - ((int32_t)T1))) >> 12) *
              ((int32_t)T3)) >> 14;
    t_preq = var1 + var2;
    return (t_preq * 5 + 128) >> 8;
}

static uint32_t Compensate_Pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_preq) - 128000;
    var2 = var1 * var1 * (int64_t)P6;
    var2 = var2 + ((var1 * (int64_t)P5) << 17);
    var2 = var2 + (((int64_t)P4) << 35);
    var1 = ((var1 * var1 * (int64_t)P3) >> 8) + ((var1 * (int64_t)P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)P1) >> 33;
    if (var1 == 0) return 0;
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)P7) << 4);
    return (uint32_t)p / 256;
}

static void Sensor_Init(void)
{
    SPI_WriteByte(BMP280_REG_RESET, 0xB6);
    HAL_Delay(10);
    /*
     * 0xF5 config: filter[2:0]=100 = x16, t_sb[2:0]=000 = 0.5ms
     * 0x57 ctrl_meas: osrs_t=010(x2), osrs_p=101(x16), mode=11(Normal)
     */
    SPI_WriteByte(BMP280_REG_CONFIG,   0x10);
    SPI_WriteByte(BMP280_REG_CTRL_MEAS, 0x57);
}

static void Read_TempPressure(int32_t *temp_c100, uint32_t *press_pa)
{
    uint8_t raw[6];
    SPI_ReadBurst(BMP280_REG_PRESS_MSB, raw, 6);

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);

    /* Temperature MUST be compensated first — sets t_preq for pressure */
    *temp_c100 = Compensate_Temperature(adc_T);
    *press_pa  = Compensate_Pressure(adc_P);
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
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CS_HIGH();
  HAL_Delay(10);

  char msg[512];

  HAL_UART_Transmit(&huart2,
      (uint8_t *)"\r\n"
      "============================================================\r\n"
      "              BMP280 ENVIRONMENT MONITOR\r\n"
      "============================================================\r\n"
      " MCU        : STM32F446RE\r\n"
      " Interface  : SPI1\r\n"
      " Sensor     : BMP280/BME280\r\n"
      "============================================================\r\n",
      185, HAL_MAX_DELAY);

  uint8_t chip_id = SPI_ReadByte(BMP280_REG_CHIP_ID);

  if (chip_id == CHIP_ID_BME280)
      HAL_UART_Transmit(&huart2, (uint8_t *)" Device Detected : BME280\r\n", 27, HAL_MAX_DELAY);
  else if (chip_id == CHIP_ID_BMP280)
      HAL_UART_Transmit(&huart2, (uint8_t *)" Device Detected : BMP280\r\n", 27, HAL_MAX_DELAY);
  else {
      snprintf(msg, sizeof(msg),
               "\r\n ERROR: Unknown Device\r\n Chip ID : 0x%02X\r\n Check SPI Connections\r\n",
               chip_id);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
      while (1);
  }

  Load_Calibration();
  Sensor_Init();

  HAL_UART_Transmit(&huart2,
      (uint8_t *)" Calibration Data Loaded Successfully\r\n"
                 " Sensor Initialization Complete\r\n"
                 "============================================================\r\n"
                 " Starting Measurements...\r\n"
                 "============================================================\r\n",
      167, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE BEGIN 3 */
	  int32_t  temp;
	  uint32_t press;

	  Read_TempPressure(&temp, &press);

	  int32_t  t_int  = temp / 100;
	  int32_t  t_frac = (temp < 0 ? -temp : temp) % 100;
	  uint32_t p_int  = press / 100;
	  uint32_t p_frac = press % 100;

	  snprintf(msg, sizeof(msg),
	           "\r\n"
	           "+----------------------------------------------------------+\r\n"
	           "|                  SENSOR LIVE DATA                        |\r\n"
	           "+----------------------------------------------------------+\r\n"
	           "| Temperature : %3ld.%02ld C                               |\r\n"
	           "| Pressure    : %4lu.%02lu hPa                             |\r\n"
	           "+----------------------------------------------------------+\r\n",
	           (long)t_int,  (long)t_frac,
	           (unsigned long)p_int, (unsigned long)p_frac);

	  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 89;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP280_CS_GPIO_Port, BMP280_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP280_CS_Pin */
  GPIO_InitStruct.Pin = BMP280_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BMP280_CS_GPIO_Port, &GPIO_InitStruct);

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
