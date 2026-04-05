#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER	128
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile char RING[BUFFER];
uint8_t RX_BYTE;
volatile uint8_t HEAD = 0;
volatile uint8_t TAIL = 0;
volatile uint8_t LINE_OK = 0;
volatile uint8_t RING_OK = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  char *msg = "LAB 04 (HAL): Ring Buffer -> Type and press Enter:\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
  HAL_UART_Receive_IT(&huart2, &RX_BYTE, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (LINE_OK)
	{
	  LINE_OK = 0; // reset flag

	  char STRING[BUFFER];
	  uint8_t i = 0;

	  // Drain buffer until empty
	  while (TAIL != HEAD)
	  {
	    STRING[i++] = RING[TAIL];
	    TAIL = (TAIL + 1) % BUFFER;
	    if (i >= BUFFER - 1) break;
	  }
	  STRING[i] = '\0';

	  // Echo back
	  char FINAL[BUFFER + 13];
	  snprintf(FINAL, sizeof(FINAL), "You Typed: %s\r\n", STRING);
	  HAL_UART_Transmit(&huart2, (uint8_t*)FINAL, strlen(FINAL), HAL_MAX_DELAY);
	}
	else if (RING_OK)
	{
	  RING_OK = 0;
	  char STRING[BUFFER];
	  uint8_t i = 0;

	  // Dump ring buffer contents
	  for (uint8_t idx = 0; idx < BUFFER || RING[idx] != '\0'; idx++)
	  {
	    STRING[i++] = RING[idx];
	  }
	  STRING[i] = '\0';

	  char FINAL[BUFFER + 15];
	  snprintf(FINAL, sizeof(FINAL), "Ring Buffer: %s\r\n", STRING);
	  HAL_UART_Transmit(&huart2, (uint8_t*)FINAL, strlen(FINAL), HAL_MAX_DELAY);
	}
	__WFI(); // sleep until next interrupt
    /* USER CODE END WHILE */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
    while(1);
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
	while(1);
  }
}

static void MX_USART2_UART_Init(void)
{
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
	while(1);
  }
}

static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart -> Instance == USART2)
    {
        char c = (char)RX_BYTE;

        // Handle Overrun Error (ORE)
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
        {
            __HAL_UART_CLEAR_OREFLAG(huart); // clear ORE gracefully and discard missing byte
        }

        if (c == '+')
        {
            RING_OK = 1; // flag for ring buffer dump
        }
        else if (c == '\r')
        {
             LINE_OK = 1; // flag for line echo
        }
        else
        {
			uint8_t next = (HEAD + 1) % BUFFER;
			if (next != TAIL)
			{
			   RING[HEAD] = c;
			   HEAD = next;
			}
        }

        // Re-arm reception
        HAL_UART_Receive_IT(&huart2, &RX_BYTE, 1);
    }
}
