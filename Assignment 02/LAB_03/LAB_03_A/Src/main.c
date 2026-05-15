#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define APB1_Clock	45000000UL
#define BAUD_RATE	115200UL
#define MAIN		main

char str0[120] = "[#] | Block Description                       |  Cycles  |    ns     |   us   |  ms  |  s  |  Baud  \r\n";
char str1[120] = "[1] | Bubble Sort (n = 100) in DWT            |          |           |        |      |     | -\r\n";
char str2[120] = "    | Bubble Sort (n = 100) in TIM            | -        |           |        |      |     | -\r\n";
char str3[120] = "[2] | delay_ms(100) in DWT                    |          |           |        |      |     | -\r\n";
char str4[120] = "    | delay_ms(100) in TIM                    | -        |           |        |      |     | -\r\n";
char str5[120] = "[3] | USART2_SendString(48) in DWT            |          |           |        |      |     |        \r\n";
char str6[120] = "    | USART2_SendString(48) in TIM            | -        |           |        |      |     |        \r\n";
char str7[120] = "[4] | Integer sq.rt. via Binary Search in DWT |          |           |        |      |     | -\r\n";
char str8[120] = "    | Integer sq.rt. via Binary Search in TIM | -        |           |        |      |     | -\r\n";
char str9[120] = "[5] | Byte-by-Byte Memory Copy in DWT         |          |           |        |      |     | -\r\n";
char str10[120]= "    | Byte-by-Byte Memory Copy in TIM         | -        |           |        |      |     | -\r\n";

void PLL_CONFIG(void)
{
	// initialize the Internal High-Speed Oscillator as the Clock for PLL (Phase-Locked Loop)
	RCC -> CR |= RCC_CR_HSION;
	// wait until HSI is initialized
	while (!(RCC -> CR & RCC_CR_HSIRDY));

	// Configuration of PLL
	RCC -> PLLCFGR =
			(16 << RCC_PLLCFGR_PLLM_Pos) | // 16MHz HSI -> 1MHz (Pre-scaler)
			(360 << RCC_PLLCFGR_PLLN_Pos) | // 1MHz to 360MHz (Multiplier)
			(0 << RCC_PLLCFGR_PLLP_Pos) | // 360 to 180MHz -> PLL_CLK
			RCC_PLLCFGR_PLLSRC_HSI; // Base Clock Source -> HSI

	// initialize the PLL for operation
	RCC -> CR |= RCC_CR_PLLON;
	while (!(RCC -> CR & RCC_CR_PLLRDY));

	// Configure Flash Latency for PLL_CLK = 180MHz at Access Control Register
	FLASH -> ACR =
			FLASH_ACR_ICEN | // Instruction Cache Enable
			FLASH_ACR_DCEN | // Data Cache Enable
			FLASH_ACR_LATENCY_5WS; // Latency of 5 Wait States as per state cover approximately 30MHz

	// Configuring Pre-scalers
	RCC -> CFGR |=
			RCC_CFGR_HPRE_DIV1 | // PLL_CLK -> AHB Bus -> 180MHz
			RCC_CFGR_PPRE1_DIV4 | // PLL_CLK -> APB1 Bus -> 45MHz
			RCC_CFGR_PPRE2_DIV2; // PLL_CLK -> APB2 Bus -> 90MHz

	RCC -> CFGR |= RCC_CFGR_SW_PLL; // Switch SYSCLK from HSI to PLL
	while ((RCC -> CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // wait until switch complete

}

void USART2_CONFIG(void)
{
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // clock sent to GPIOA Ports
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN; // clock sent to USART2 Ports
	__NOP(); // delay for clock initiation

	GPIOA -> MODER &= ~((3UL << 2*2) | (3UL << 3*2)); // Reset MODE Register of GPIOA of PA2, PA3
	GPIOA -> MODER |= ((2UL << 2*2) | (2UL << 3*2)); // Set MODE Register to Alternate-Function Mode for PA2(TX), PA3(RX)

	GPIOA -> AFR[0] &= ~((0xF << 2*4) | (0xF << 3*4)); // Clearing the AFR Register Low for PA2, PA3 (each take 4 bits)
	GPIOA -> AFR[0] |= ((7UL << 2*4) | (7UL << 3*4)); // Set the AFR Register Low in PA2, PA3 Position to USART2 TX/RX

	GPIOA -> OTYPER &= ~((1UL << 2) | (1UL << 3)); // Set output type to push-pull -> 0b0
	GPIOA -> OSPEEDR |= ((3UL << 2*2) | (3UL << 3*2)); // Set Output Speed at very high -> 0b11
	GPIOA -> PUPDR &= ~((3UL << 2*2) | (3UL << 3*2)); // Set pull-up pull down to no pull-up no pull-down -> 0b00

	/*
	USARTDIV = 45000000 / (16 * 115200)
	         ≈ 24.4140625
	Mantissa = 24
	Fraction = 0.4140625 * 16 ≈ 7
	*/
	USART2 -> BRR = (24 << 4) | 7;

	USART2 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; // Enable USART2
}

void USART2_SEND_CHAR(char C)
{
	while (!(USART2 -> SR & USART_SR_TXE));
	USART2 -> DR = (uint8_t) C;
}

void USART2_SEND_STRING(char* str)
{
	while(*str) USART2_SEND_CHAR(*str++);
}

void DWT_CONFIG(void) // Data Watch-point and Trace
{
	CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enabling the Trace Enable Control, DEMCR = Debug Exception and Monitor Control Register
	DWT -> CYCCNT = 0; // Resetting Cycle Counter
	DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enabling the CYCCNTENA bit for starting count
}

void TIM2_CONFIG(void)
{
	RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN; // TIM2 clock = 90MHz (APB1=45MHz, doubled because pre-scaler > 1)
	TIM2 -> CR1 &= ~TIM_CR1_CEN; // Disabling the TIM2 Counter

	TIM2 -> PSC = 89; // Setting PSC = 89 so that each tick becomes 1us
	TIM2 -> ARR = ~0x0; // Setting ARR to max value

	TIM2 -> EGR |= TIM_EGR_UG; // This updates the new PSC and ARR values into shadow registers, works immediately
	TIM2 -> CR1 |= TIM_CR1_CEN; // Enable the clock
}

uint32_t DWT_TIME(uint32_t cycles, uint32_t factor)
{
    double seconds = cycles / 180000000.0; // As SYSCLK = 180 MHz
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
	memcpy(&str[71], temp, strlen(temp));

	uint32_t t_millis = DWT_TIME(cycles, 1e3);
	CONVERTER(t_millis, temp);
	memcpy(&str[80], temp, strlen(temp));

	uint32_t t_secs = DWT_TIME(cycles, 1);
	CONVERTER(t_secs, temp);
	memcpy(&str[87], temp, strlen(temp));
}

void CALC_TIM(char* str, uint32_t micros)
{
	char temp[16];

	uint32_t t_nanos = TIM_TIME(micros, 1e9);
	CONVERTER(t_nanos, temp);
	memcpy(&str[59], temp, strlen(temp));

	uint32_t t_micros = TIM_TIME(micros, 1e6);
	CONVERTER(t_micros, temp);
	memcpy(&str[71], temp, strlen(temp));

	uint32_t t_millis = TIM_TIME(micros, 1e3);
	CONVERTER(t_millis, temp);
	memcpy(&str[80], temp, strlen(temp));

	uint32_t t_secs = TIM_TIME(micros, 1);
	CONVERTER(t_secs, temp);
	memcpy(&str[87], temp, strlen(temp));
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

void DELAY_MS(uint32_t ms)
{
	uint32_t INIT = TIM2 -> CNT;
	while ((TIM2 -> CNT - INIT) / 1000 != ms);
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
	uint32_t T1 = TIM2 -> CNT;
	BUBBLE_SORT(arr);
	uint32_t T2 = TIM2 -> CNT;
	uint32_t micros = T2 - T1;
	CALC_TIM(str2, micros);
}

void PROFILE_02_DWT(void)
{
	uint32_t T1 = DWT -> CYCCNT;
	DELAY_MS(100);
	uint32_t T2 = DWT -> CYCCNT;
	uint32_t cycles = T2 - T1;
	CALC_DWT(str3, cycles);
}

void PROFILE_02_TIM(void)
{
	TIM2 -> CNT = 0;
	uint32_t T1 = TIM2 -> CNT;
	DELAY_MS(100);
	uint32_t T2 = TIM2 -> CNT;
	uint32_t micros = T2 - T1;
	CALC_TIM(str4, micros);
}

void PROFILE_03_DWT(void)
{
	char* c = "Profiling code speed at 180MHz clock ticks....\r\n";
	uint32_t T1 = DWT -> CYCCNT;
	USART2_SEND_STRING(c);
	uint32_t T2 = DWT -> CYCCNT;
	uint32_t cycles = T2 - T1;
	char t[10];
	CALC_DWT(str5, cycles);
	double micros = cycles / 180.0;
	micros = 480 / micros; // 8 bit sending, 1 start bit, 1/2 stop bits, generally 10 bits
	micros *= 1e6;
	CONVERTER((uint32_t)micros, t);
	memcpy(&str5[93], t, strlen(t));
}

void PROFILE_03_TIM(void)
{
	char* c = "Profiling code speed at 180MHz clock ticks....\r\n";
	TIM2 -> CNT = 0;
	uint32_t T1 = TIM2 -> CNT;
	USART2_SEND_STRING(c);
	uint32_t T2 = TIM2 -> CNT;
	uint32_t micros = T2 - T1;
	char t[10];
	CALC_TIM(str6, micros);
	double micro = 480.0 / micros; // 8 bit sending, 1 start bit, 1/2 stop bits, generally 10 bits
	micro *= 1e6;
	CONVERTER((uint32_t)micro, t);
	memcpy(&str6[93], t, strlen(t));
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
	uint32_t T1 = TIM2 -> CNT;
	for (int i = 1; i <= 1000; i++) SQRT_BINARY(i);
	uint32_t T2 = TIM2 -> CNT;
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
	uint32_t T1 = TIM2 -> CNT;
	for (int i = 0; i < 512; i++) dest[i] = src[i];
	uint32_t T2 = TIM2 -> CNT;
	uint32_t micros = T2 - T1;
	CALC_TIM(str10, micros);
}

void TABLE_OUT(void)
{
	USART2_SEND_STRING(str0);
	USART2_SEND_STRING(str1);
	USART2_SEND_STRING(str2);
	USART2_SEND_STRING(str3);
	USART2_SEND_STRING(str4);
	USART2_SEND_STRING(str5);
	USART2_SEND_STRING(str6);
	USART2_SEND_STRING(str7);
	USART2_SEND_STRING(str8);
	USART2_SEND_STRING(str9);
	USART2_SEND_STRING(str10);
}

int MAIN(void)
{
	// System Configurations
	PLL_CONFIG();
	USART2_CONFIG();
	DWT_CONFIG();
	TIM2_CONFIG();

	char* STRING = "LAB 03 (Bare-Metal): Duration Measurement with Code Profiling using USART\r\n\n";
	USART2_SEND_STRING(STRING);

	char* c1 = "Task 01: Bubble Sort (Worst Case) for n = 100\r\n";
	USART2_SEND_STRING(c1);
	PROFILE_01_DWT();
	PROFILE_01_TIM();

	char* c2 = "Task 02: delay_ms(100) for DWT and TIM\r\n";
	USART2_SEND_STRING(c2);
	PROFILE_02_DWT();
	PROFILE_02_TIM();

	char* c3 = "Task 03: USART2 48-Byte Transmission Check\r\n";
	USART2_SEND_STRING(c3);
	PROFILE_03_DWT();
	PROFILE_03_TIM();

	char* c4 = "Task 04: Integer Square Root check by Binary Search for n = 1000\r\n";
	USART2_SEND_STRING(c4);
	PROFILE_04_DWT();
	PROFILE_04_TIM();

	char* c5 = "Task 05: Memory Copy of 512 Bytes byte-by-byte\r\n";
	USART2_SEND_STRING(c5);
	PROFILE_05_DWT();
	PROFILE_05_TIM();

	char* f = "\nFinal Calculation\r\n\n";
	USART2_SEND_STRING(f);
	TABLE_OUT();
}
