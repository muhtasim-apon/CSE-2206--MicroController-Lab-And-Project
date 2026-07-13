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

/* 4. Structure for Calibration: */

typedef struct {
    uint16_t TEMP_1;
    int16_t  TEMP_2;
    int16_t  TEMP_3;
    uint16_t PRESS_1;
    int16_t  PRESS_2;
    int16_t  PRESS_3;
    int16_t  PRESS_4;
    int16_t  PRESS_5;
    int16_t  PRESS_6;
    int16_t  PRESS_7;
    int16_t  PRESS_8;
    int16_t  PRESS_9;
    uint8_t  HUM_1;
    int16_t  HUM_2;
    uint8_t  HUM_3;
    int16_t  HUM_4;
    int16_t  HUM_5;
    int8_t   HUM_6;
} BME_P280_CALIBRATION;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 2. Register Mapping: */

#define BME_P280_REGISTER_CHIP_ID      				0xD0
#define BME_P280_REGISTER_RESET        				0xE0
#define BME_P280_REGISTER_CTRL_HUM   				0xF2
#define BME_P280_REGISTER_STATUS       				0xF3
#define BME_P280_REGISTER_CONTROL_MEASURE    		0xF4
#define BME_P280_REGISTER_CONFIGURE       			0xF5
#define BME_P280_REGISTER_PRESSURE_MSB    			0xF7
#define BME_P280_REGISTER_HUM_MSB     				0xFD
#define BME_P280_REGISTER_HUM_LSB     				0xFE
#define BME_P280_REGISTER_CALIBRATION_START  		0x88
#define BME_P280_REGISTER_HUM_CALIB   				0xA1   // extra calibration byte
#define BME_P280_REGISTER_HUM_CALIB2  				0xE1   // block of humidity calibration
#define BME_P280_RESET_VALUE      					0xB6
#define BME_P280_CTRL_HUM_VALUE      				0x01   // oversampling x1 (minimum)
#define BME_P280_CONTROL_MEASURE_VALUE    			((2U << 5) | (5U << 2) | 3U)   // Decodes to 0x57
#define BME_P280_CONFIGURE_VALUE       				((4U << 5) | (4U << 2))        // Decodes to 0x90

/* 3. Address Mapping: */

#define BME_P280_I2C_ADDRESS         				0x76
#define CHIP_ADDRESS_BME280          				0x60
#define CHIP_ADDRESS_BMP280          				0x58

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static BME_P280_CALIBRATION CALIBRATION;
static int32_t t_fine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void USART2_SendString(const char *s);
void delay_us(uint16_t us);
void delay_ms(uint32_t ms);
void I2C_WRITE_BYTE(uint8_t reg, uint8_t data);
uint8_t I2C_READ_BYTE(uint8_t reg);
void I2C_READ_BYTES(uint8_t start_reg, uint8_t *buf, uint8_t len);
int I2C_DEVICE_ALIVE(void);
void BME_P280_CALIBRATION_READ(void);
void BME_P280_INIT(void);
int32_t BME_P280_BOSCH_TEMP_COMPENSATE(int32_t adc_T);
uint32_t BME_P280_BOSCH_PRESS_COMPENSATE(int32_t adc_P);
uint32_t BME_P280_BOSCH_HUM_COMPENSATE(int32_t adc_H);
void BME_P280_TEMP_PRESS_HUM(void);
void SENSOR_CHECK(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void USART2_SendString(const char *s)
{
    // Blocking transmit of a null-terminated string over USART2 via HAL
    HAL_UART_Transmit(&huart2, (uint8_t *)s, strlen(s), HAL_MAX_DELAY);
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0U); // Reset TIM6 counter
    while ((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) < us); // Wait until the counter reaches us ticks (1us per tick)
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        delay_us(1000U);
    }
}

void I2C_WRITE_BYTE(uint8_t reg, uint8_t data)
{
    // Write a single byte to a register using HAL memory-write (handles START/ADDR/STOP internally)
    HAL_I2C_Mem_Write(&hi2c1, (BME_P280_I2C_ADDRESS << 1), reg,
                      I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

uint8_t I2C_READ_BYTE(uint8_t reg)
{
    uint8_t val = 0xFF; // Buffer for Read value

    // Read a single byte from a register using HAL memory-read (handles repeated START + NACK internally)
    HAL_I2C_Mem_Read(&hi2c1, (BME_P280_I2C_ADDRESS << 1), reg,
                     I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);

    return val;
}

void I2C_READ_BYTES(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    if (len == 0) return;

    // Burst read of len bytes starting at start_reg using HAL memory-read (auto ACK/NACK per byte)
    HAL_I2C_Mem_Read(&hi2c1, (BME_P280_I2C_ADDRESS << 1), start_reg,
                     I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);
}

int I2C_DEVICE_ALIVE(void)
{
    // Probe the slave address; returns 1 if the device ACKs within the given trials/timeout
    int alive = (HAL_I2C_IsDeviceReady(&hi2c1, (BME_P280_I2C_ADDRESS << 1), 3, 100) == HAL_OK); // Boolean to check if Slave ACKs
    delay_ms(2);
    return alive;
}

void BME_P280_CALIBRATION_READ(void)
{
    uint8_t CALIB_VAL[24];
    I2C_READ_BYTES(BME_P280_REGISTER_CALIBRATION_START, CALIB_VAL, 24); // Load 24-byte calibration block starting at 0x88

    CALIBRATION.TEMP_1 = (uint16_t)((CALIB_VAL[1]  << 8) | CALIB_VAL[0]);
    CALIBRATION.TEMP_2 = (int16_t) ((CALIB_VAL[3]  << 8) | CALIB_VAL[2]);
    CALIBRATION.TEMP_3 = (int16_t) ((CALIB_VAL[5]  << 8) | CALIB_VAL[4]);

    CALIBRATION.PRESS_1 = (uint16_t)((CALIB_VAL[7]  << 8) | CALIB_VAL[6]);
    CALIBRATION.PRESS_2 = (int16_t) ((CALIB_VAL[9]  << 8) | CALIB_VAL[8]);
    CALIBRATION.PRESS_3 = (int16_t) ((CALIB_VAL[11] << 8) | CALIB_VAL[10]);
    CALIBRATION.PRESS_4 = (int16_t) ((CALIB_VAL[13] << 8) | CALIB_VAL[12]);
    CALIBRATION.PRESS_5 = (int16_t) ((CALIB_VAL[15] << 8) | CALIB_VAL[14]);
    CALIBRATION.PRESS_6 = (int16_t) ((CALIB_VAL[17] << 8) | CALIB_VAL[16]);
    CALIBRATION.PRESS_7 = (int16_t) ((CALIB_VAL[19] << 8) | CALIB_VAL[18]);
    CALIBRATION.PRESS_8 = (int16_t) ((CALIB_VAL[21] << 8) | CALIB_VAL[20]);
    CALIBRATION.PRESS_9 = (int16_t) ((CALIB_VAL[23] << 8) | CALIB_VAL[22]);

    CALIBRATION.HUM_1 = I2C_READ_BYTE(BME_P280_REGISTER_HUM_CALIB);
	uint8_t humCalib[7];
	I2C_READ_BYTES(BME_P280_REGISTER_HUM_CALIB2, humCalib, 7);

	CALIBRATION.HUM_2 = (int16_t)((humCalib[1] << 8) | humCalib[0]);
	CALIBRATION.HUM_3 = humCalib[2];
	CALIBRATION.HUM_4 = (int16_t)((humCalib[3] << 4) | (humCalib[4] & 0x0F));
	CALIBRATION.HUM_5 = (int16_t)((humCalib[5] << 4) | (humCalib[4] >> 4));
	CALIBRATION.HUM_6 = (int8_t)humCalib[6];
}

void BME_P280_INIT(void)
{
    // 1. Reset the sensor (soft reset command 0xB6 to register 0xE0)
    I2C_WRITE_BYTE(BME_P280_REGISTER_RESET, BME_P280_RESET_VALUE);
    delay_ms(10); // Wait for reset to complete

    // 2. Configure general settings (IIR filter, standby time, etc.)
    // Value 0x90 = filter coefficient x16, standby time 125 ms
    I2C_WRITE_BYTE(BME_P280_REGISTER_CONFIGURE, BME_P280_CONFIGURE_VALUE);
    delay_ms(2); // Short delay for register update
    I2C_WRITE_BYTE(BME_P280_REGISTER_CTRL_HUM, BME_P280_CTRL_HUM_VALUE);
    delay_ms(2);

    // 3. Configure measurement control (oversampling + mode)
    // Value 0x57 = Temp oversampling x2, Pressure oversampling x5, Normal mode
    I2C_WRITE_BYTE(BME_P280_REGISTER_CONTROL_MEASURE, BME_P280_CONTROL_MEASURE_VALUE);
    delay_ms(2); // Short delay for register update
}

int32_t BME_P280_BOSCH_TEMP_COMPENSATE(int32_t adc_T)
{
    int32_t var1, var2, T;

    // First correction term: linear offset/slope adjustment
    var1 = ((((adc_T >> 3) - ((int32_t)CALIBRATION.TEMP_1 << 1)))
             * ((int32_t)CALIBRATION.TEMP_2)) >> 11;

    // Second correction term: quadratic correction for non-linearity
    var2 = (((((adc_T >> 4) - ((int32_t)CALIBRATION.TEMP_1))
             * ((adc_T >> 4) - ((int32_t)CALIBRATION.TEMP_1))) >> 12)
             * ((int32_t)CALIBRATION.TEMP_3)) >> 14;

    // Fine temperature value (global correction factor reused in pressure/humidity)
    t_fine = var1 + var2;

    // Final compensated temperature in 0.01 °C units
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

uint32_t BME_P280_BOSCH_PRESS_COMPENSATE(int32_t adc_P)
{
    int64_t var1, var2, p;

    // Fine temperature correction
    var1 = ((int64_t)t_fine) - 128000;

    // Higher-order polynomial corrections
    var2 = var1 * var1 * (int64_t)CALIBRATION.PRESS_6;
    var2 = var2 + ((var1 * (int64_t)CALIBRATION.PRESS_5) << 17);
    var2 = var2 + (((int64_t)CALIBRATION.PRESS_4) << 35);

    // More corrections using PRESS_2 and PRESS_3
    var1 = ((var1 * var1 * (int64_t)CALIBRATION.PRESS_3) >> 8)
         + ((var1 * (int64_t)CALIBRATION.PRESS_2) << 12);

    // Scaling with PRESS_1 (main pressure calibration factor)
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)CALIBRATION.PRESS_1) >> 33;

    // Guard against division by zero
    if (var1 == 0) return 0;

    // Pressure calculation
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    // PRESS_9 and PRESS_8 corrections
    var1 = (((int64_t)CALIBRATION.PRESS_9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)CALIBRATION.PRESS_8) * p) >> 19;

    // Final adjustment with PRESS_7
    p = ((p + var1 + var2) >> 8) + (((int64_t)CALIBRATION.PRESS_7) << 4);

    // Return compensated pressure in Pa (integer)
    return (uint32_t)(p >> 8);
}

uint32_t BME_P280_BOSCH_HUM_COMPENSATE(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)CALIBRATION.HUM_4) << 20)
                  - (((int32_t)CALIBRATION.HUM_5) * v_x1_u32r)) + ((int32_t)16384)) >> 15)
                  * ((((((v_x1_u32r * ((int32_t)CALIBRATION.HUM_6)) >> 10)
                  * (((v_x1_u32r * ((int32_t)CALIBRATION.HUM_3)) >> 11) + ((int32_t)32768))) >> 10)
                  + ((int32_t)2097152)) * ((int32_t)CALIBRATION.HUM_2) + 8192)) >> 14;

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                  * ((int32_t)CALIBRATION.HUM_1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12); // result in %RH * 1024
}

void BME_P280_TEMP_PRESS_HUM(void)
{
    char MESSAGE[120];
    uint8_t VALUE[8];

    I2C_READ_BYTES(BME_P280_REGISTER_PRESSURE_MSB, VALUE, 8);

    int32_t P = ((int32_t)VALUE[0] << 12) | ((int32_t)VALUE[1] << 4) | ((int32_t)VALUE[2] >> 4);
    int32_t T = ((int32_t)VALUE[3] << 12) | ((int32_t)VALUE[4] << 4) | ((int32_t)VALUE[5] >> 4);
    int32_t H = ((int32_t)VALUE[6] << 8) | VALUE[7];

    int32_t TEMP_BASIC = BME_P280_BOSCH_TEMP_COMPENSATE(T);
    uint32_t PRESSURE_PASCAL = BME_P280_BOSCH_PRESS_COMPENSATE(P);
    uint32_t HUMIDITY_BASIC = BME_P280_BOSCH_HUM_COMPENSATE(H);

    int32_t TEMP_FINAL = TEMP_BASIC / 100;
    int32_t TEMP_FRACTION = (TEMP_BASIC < 0 ? -TEMP_BASIC : TEMP_BASIC) % 100;
    uint32_t PRESSURE_HECTO = PRESSURE_PASCAL / 100;
    uint32_t PRESSURE_HECTO_FRACTION = PRESSURE_PASCAL % 100;
    uint32_t HUMIDITY_FINAL = HUMIDITY_BASIC / 1024;
    uint32_t HUMIDITY_FRACTION = (HUMIDITY_BASIC % 1024) * 100 / 1024;

    sprintf(MESSAGE, "|  %6ld.%02ld°C       |  %6lu.%02lu hPa   |   %3lu.%02lu %%RH     |\r\n",
            (long)TEMP_FINAL, (long)TEMP_FRACTION,
            (unsigned long)PRESSURE_HECTO, (unsigned long)PRESSURE_HECTO_FRACTION,
            (unsigned long)HUMIDITY_FINAL, (unsigned long)HUMIDITY_FRACTION);

    USART2_SendString(MESSAGE);
}

void SENSOR_CHECK(void)
{
    char MESSAGE[80];

    USART2_SendString("============================================================\r\n");
    USART2_SendString("[1] Sending Slave Address (0x76) and checking ACK...\r\n");
    if (!I2C_DEVICE_ALIVE()) {
        USART2_SendString("[!] ACK not received. Going idle...\r\n");
        while (1);
    }
    USART2_SendString("[*] Device ACK received...\r\n");
    delay_ms(5000);

    USART2_SendString("============================================================\r\n");
    USART2_SendString("[2] Reading chip ID register (0xD0)...\r\n");
    uint8_t id = I2C_READ_BYTE(BME_P280_REGISTER_CHIP_ID);
    sprintf(MESSAGE, "Raw value: 0x%02X\r\n", id);
    USART2_SendString(MESSAGE);
    USART2_SendString("============================================================\r\n");

    USART2_SendString("[3] Identifying the sensor (BME280 or BMP280)...\r\n");
    if (id == CHIP_ADDRESS_BME280)
        USART2_SendString("[*] BME280 detected (has humidity)\r\n");
    else if (id == CHIP_ADDRESS_BMP280)
    	USART2_SendString("[*] BMP280 detected\r\n");
    else
    {
        sprintf(MESSAGE, "[!] UNKNOWN chip ID: 0x%02X.. Going Idle...\r\n", id);
        USART2_SendString(MESSAGE);
        while (1);
    }
    USART2_SendString("============================================================\r\n");
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
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim6); // Start TIM6 free-running so delay_us()/delay_ms() work (1us per tick)

  USART2_SendString("LAB 02 (HAL): BME/P-280 Sensor Communication by I2C\r\n");
  delay_ms(5000);
  SENSOR_CHECK();

  USART2_SendString("[4] Reading Chip Calibration Factors...\r\n");
  BME_P280_CALIBRATION_READ();
  USART2_SendString("Calibration Factors found and loaded...\r\n");
  USART2_SendString("============================================================\r\n");

  USART2_SendString("[5] Configuring BME/P280 in Normal Mode...\r\n");
  BME_P280_INIT();
  USART2_SendString("BME/P Ready for Operation...\r\n");
  delay_ms(5000);

  USART2_SendString("============================================================\r\n");
  USART2_SendString("[6] Temperature and Pressure Record per Second...\r\n");
  USART2_SendString("============================================================\r\n");
  USART2_SendString("|  Temperature (°C)  |  Pressure (hPa)  |  Humidity (%RH)  |\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    BME_P280_TEMP_PRESS_HUM();
    delay_ms(1000);
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.Mode = UART_MODE_TX;
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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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
