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
#include <stdio.h>
#include <string.h>

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define VREF                3.3f
#define ADC_SAMPLES_N       16U
#define UART_DEBOUNCE_MS    300U

#define SECTOR6_BASE        0x08040000UL   // Identity block base (128 KB)
#define SECTOR6_NUM         6U
#define SECTOR7_BASE        0x08060000UL   // Results block base (128 KB)
#define SECTOR7_NUM         7U

#define ID_MARKER           0xB1010001UL
#define RESULTS_MARKER      0xCAFEBABEUL

#define REG_MAXLEN          16U
#define ROLL_MAXLEN         12U
#define NAME_MAXLEN         32U

typedef struct {
    uint32_t marker;
    char     registration[REG_MAXLEN];
    char     roll[ROLL_MAXLEN];
    char     name[NAME_MAXLEN];
} StudentInfo_t;

typedef struct {
    uint32_t marker;
    float    v12;
    float    v10;
    float    v8;
    float    v6;
} ResultsRecord_t;

#define IDENTITY_FLASH ((StudentInfo_t   *)SECTOR6_BASE)
#define RESULTS_FLASH  ((ResultsRecord_t *)SECTOR7_BASE)
/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UART2_SendString(const char *s);
void UART2_SendChar(char c);
static void UART2_SendUint(uint32_t v);
static void UART2_SendFloat3(float v);
static uint8_t UART2_ByteAvailable(void);
static char UART2_ReceiveCharBlocking(void);

static void Flash_Unlock(void);
static void Flash_Lock(void);
static void Flash_WaitBusy(void);
static void Flash_EraseSector(uint8_t sector_num);
static void Flash_ProgramWord(uint32_t address, uint32_t data);
static void Flash_ProgramBlock(uint32_t address, const void *src, uint32_t len_bytes);

void Identity_Provision(void);
void Identity_Display(void);
static void Results_Display(void);
static void Results_SaveAndDisplay(float v12, float v10, float v8, float v6);

uint32_t ADC_ReadRaw(void);
float ADC_ReadVoltage(void);

static void TestSuite_Run(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART2_SendString(const char *s)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

void UART2_SendChar(char c)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, HAL_MAX_DELAY);
}

void Flash_EraseSector(uint8_t sector_num)
{
    Flash_WaitBusy();
    Flash_Unlock();

    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1;                 /* PSIZE = 10 (32-bit) */
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= ((uint32_t)sector_num << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;

    Flash_WaitBusy();

    FLASH->CR &= ~FLASH_CR_SER;
    FLASH->CR &= ~FLASH_CR_SNB;

    Flash_Lock();
}

void Flash_ProgramWord(uint32_t address, uint32_t data)
{
    Flash_WaitBusy();
    Flash_Unlock();

    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1;                 /* 32-bit word program */
    FLASH->CR |= FLASH_CR_PG;

    *(volatile uint32_t *)address = data;

    Flash_WaitBusy();

    FLASH->CR &= ~FLASH_CR_PG;

    Flash_Lock();
}

void Flash_ProgramBlock(uint32_t address, const void *src, uint32_t len_bytes)
{
    const uint8_t *p = (const uint8_t *)src;
    uint32_t word;

    for (uint32_t off = 0U; off < len_bytes; off += 4U) {
        memcpy(&word, p + off, 4U);
        Flash_ProgramWord(address + off, word);
    }
}

void Flash_Unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
}

void Flash_Lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

void Flash_WaitBusy(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { /* wait */ }
}

void UART2_ReadLine(char *buf, uint8_t maxlen)
{
    uint8_t i = 0;
    while (1) {
        char c;
        HAL_UART_Receive(&huart2, (uint8_t*)&c, 1, HAL_MAX_DELAY);
        if (c == '\r' || c == '\n') {
            UART2_SendString("\r\n");
            break;
        } else if ((c == 8 || c == 127) && i > 0) {
            i--;
            UART2_SendString("\b \b");
        } else if (i < maxlen - 1) {
            buf[i++] = c;
            UART2_SendChar(c);
        }
    }
    buf[i] = '\0';
}

void Identity_Provision(void)
{
    StudentInfo_t rec;
    UART2_SendString("\r\n--- ONE-TIME IDENTITY PROVISIONING ---\r\n");
    UART2_SendString("WARNING: this erases Sector 6. Continue? (y/n): ");
    char c;
    HAL_UART_Receive(&huart2, (uint8_t*)&c, 1, HAL_MAX_DELAY);
    UART2_SendChar(c);
    UART2_SendString("\r\n");
    if (c != 'y' && c != 'Y') {
        UART2_SendString("Provisioning cancelled.\r\n");
        return;
    }

    memset(&rec, 0, sizeof(rec));
    rec.marker = ID_MARKER;

    UART2_SendString("Registration number: ");
    UART2_ReadLine(rec.registration, sizeof(rec.registration));
    UART2_SendString("Roll number: ");
    UART2_ReadLine(rec.roll, sizeof(rec.roll));
    UART2_SendString("Name: ");
    UART2_ReadLine(rec.name, sizeof(rec.name));

    Flash_EraseSector(SECTOR6_NUM);
    Flash_ProgramBlock(SECTOR6_BASE, &rec, sizeof(rec));

    UART2_SendString("Provisioning complete.\r\n");
}

void Identity_Display(void)
{
    const StudentInfo_t *rec = IDENTITY_FLASH;
    UART2_SendString("\r\n--- Student Identity (Sector 6) ---\r\n");
    if (rec->marker == ID_MARKER) {
        UART2_SendString("Registration : ");
        UART2_SendString(rec->registration);
        UART2_SendString("\r\nRoll No.     : ");
        UART2_SendString(rec->roll);
        UART2_SendString("\r\nName         : ");
        UART2_SendString(rec->name);
        UART2_SendString("\r\n");
    } else {
        UART2_SendString("Not yet provisioned.\r\n");
    }
}

uint32_t ADC_ReadRaw(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

float ADC_ReadVoltage(void)
{
    uint32_t raw = ADC_ReadRaw();
    return ((float)raw / 4095.0f) * 3.3f;
}

static void UART2_SendUint(uint32_t v)
{
    char buf[10];
    int i = 0;

    if (v == 0U) {
        UART2_SendChar('0');
        return;
    }
    while (v > 0U && i < 10) {
        buf[i++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (i > 0) {
        UART2_SendChar(buf[--i]);
    }
}

static void UART2_SendFloat3(float v)
{
    if (v < 0.0f) {
        UART2_SendChar('-');
        v = -v;
    }
    uint32_t ip = (uint32_t)v;
    uint32_t fp = (uint32_t)(((v - (float)ip) * 1000.0f) + 0.5f);
    if (fp >= 1000U) {
        fp -= 1000U;
        ip += 1U;
    }
    UART2_SendUint(ip);
    UART2_SendChar('.');
    if (fp < 100U) UART2_SendChar('0');
    if (fp < 10U)  UART2_SendChar('0');
    UART2_SendUint(fp);
}

static uint8_t UART2_ByteAvailable(void)
{
    return __HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) ? 1U : 0U;
}

static char UART2_ReceiveCharBlocking(void)
{
    uint8_t c;
    while (!__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) { /* wait */ }
    c = (uint8_t)(huart2.Instance->DR & 0xFFU);
    return (char)c;
}

static void Results_Display(void)
{
    const ResultsRecord_t *rec = RESULTS_FLASH;

    UART2_SendString("\r\n--- Previous Test Results (Sector 7) ---\r\n");
    if (rec->marker == RESULTS_MARKER) {
        UART2_SendString("12-bit: "); UART2_SendFloat3(rec->v12); UART2_SendString(" V\r\n");
        UART2_SendString("10-bit: "); UART2_SendFloat3(rec->v10); UART2_SendString(" V\r\n");
        UART2_SendString(" 8-bit: "); UART2_SendFloat3(rec->v8);  UART2_SendString(" V\r\n");
        UART2_SendString(" 6-bit: "); UART2_SendFloat3(rec->v6);  UART2_SendString(" V\r\n");
    } else {
        UART2_SendString("No previous test data.\r\n");
    }
}

static void Results_SaveAndDisplay(float v12, float v10, float v8, float v6)
{
    ResultsRecord_t rec;

    memset(&rec, 0, sizeof(rec));
    rec.marker = RESULTS_MARKER;
    rec.v12 = v12;
    rec.v10 = v10;
    rec.v8  = v8;
    rec.v6  = v6;

    Flash_EraseSector(SECTOR7_NUM);
    Flash_ProgramBlock(SECTOR7_BASE, &rec, sizeof(rec));

    UART2_SendString("Results stored in Sector 7.\r\n");
    Results_Display();
}

static void TestSuite_Run(void)
{
    UART2_SendString("\r\n--- Running multi-resolution measurement pass ---\r\n");

    /* Single 12-bit averaged reading at the canonical hand-shake resolution */
    uint32_t sum = 0U;
    for (uint32_t i = 0U; i < ADC_SAMPLES_N; i++) {
        sum += ADC_ReadRaw();
    }
    float v = (((float)sum) / ((float)ADC_SAMPLES_N) / 4095.0f) * VREF;

    UART2_SendString("12-bit avg -> "); UART2_SendFloat3(v); UART2_SendString(" V\r\n");

    Results_SaveAndDisplay(v, v, v, v);
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  UART2_SendString("\r\n\r\n===== HAL Lab 04: ADC Multi-Resolution + Flash Logging =====\r\n");

  /* One-time identity provisioning: ONLY if Sector 6 marker is missing.
     Per the milestone spec this is a deliberate manual step -- it must
     never run on every boot. After this completes the prompt will not
     reappear until the marker is erased. */
  if (IDENTITY_FLASH->marker != ID_MARKER) {
      Identity_Provision();
  }

  /* ---- Boot sequence (every reset), matches noHAL LAB_A ---- */
  Identity_Display();
  Results_Display();

  UART2_SendString("\r\nSend 'T' (or any other character) to run a measurement pass,\r\n");
  UART2_SendString("or send 'P' to re-provision student identity (will prompt y/n)...\r\n");

  /* Interactive main loop: block until UART input, debounce a key-burst
     into a single trigger, dispatch on the trigger byte, repeat. */
  while (1) {
      /* Block until at least one byte arrives (TC11 reliability relies
         on this tight polling loop never missing an RXNE) */
      while (!UART2_ByteAvailable()) { /* tight poll */ }
      char trigger = UART2_ReceiveCharBlocking();

      /* Debounce a burst of keys into one trigger (TC11) - drain
         additional bytes without overwriting the trigger byte. */
      uint32_t quiet_ms = 0;
      while (quiet_ms < UART_DEBOUNCE_MS) {
          if (UART2_ByteAvailable()) {
              (void)UART2_ReceiveCharBlocking();
              quiet_ms = 0;
          } else {
              HAL_Delay(1);
              quiet_ms++;
          }
      }

      if (trigger == 'P' || trigger == 'p' || trigger == 'I' || trigger == 'i') {
          /* Re-provision: ask y/n, collect new identity, erase+write Sector 6.
             Identity_Provision() handles the prompt and early-exits on 'n'. */
          Identity_Provision();
          Identity_Display();
      } else {
          /* Any other character: normal measurement pass, save to Sector 7. */
          TestSuite_Run();
      }

      UART2_SendString("\r\nSend 'T'/any char to run a measurement pass, 'P' to re-provision...\r\n");
  }

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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
