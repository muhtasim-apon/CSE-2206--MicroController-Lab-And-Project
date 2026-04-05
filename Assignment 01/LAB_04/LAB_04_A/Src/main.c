#include "stm32f446xx.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#define BUFFER		128
#define APB1_CLK	45000000UL
#define BAUD_RATE	115200UL
#define MAIN		main

volatile char RING[BUFFER]; // for using in ISR
volatile uint8_t LINE_OK = 0; // status flag for line show
volatile uint8_t RING_OK = 0; // status flag for ring show
volatile uint8_t HEAD = 0; // for using in ISR
volatile uint8_t TAIL = 0; // for using in main, prevents race condition

void PLL_CONFIG(void)
{
	RCC -> CR |= RCC_CR_HSION; // Enabling HSI (Internal High Speed Oscillator -> 16MHz)
	while (!(RCC -> CR & RCC_CR_HSIRDY)); // waits until HSI is being ready for operation

	RCC -> PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) // 16 as divider (PLLM)
			| (180 << RCC_PLLCFGR_PLLN_Pos) // 180 as multiplier (PLLN)
			| (0 << RCC_PLLCFGR_PLLP_Pos) // 2 as pre-scaler for PLLCLK (PLLP) -> 90MHz
			| (4 << RCC_PLLCFGR_PLLQ_Pos) // PLLQ = 4 used for USART Transmit
			| RCC_PLLCFGR_PLLSRC_HSI; // PLLCLK Source = HSI

	RCC -> CR |= RCC_CR_PLLON; // Enabling PLLCLK
	while (!(RCC -> CR & RCC_CR_PLLRDY)); // waiting for finishing

	FLASH -> ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;
	/* Flash latency set for 2 Wait States
	 * as 90MHz for SYSCLK is a bit high for
	 * instruction cycles */

	RCC -> CFGR = // OR omitted for overwriting values and setting values precisely
			RCC_CFGR_HPRE_DIV1 // AHB pre-scaler = 1
			| RCC_CFGR_PPRE1_DIV2 // APB1 pre-scaler = 2 -> 45MHz Config
			| RCC_CFGR_PPRE2_DIV1; // APB2 pre-scaler = 1

	RCC -> CFGR |= RCC_CFGR_SW_PLL; // setting PLLCLK as SYSCLK
	while ((RCC -> CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // waiting until the SYSCLK switches to PLLCLK
}

void USART2_IRQHandler(void)
{
	if (USART2 -> SR & USART_SR_RXNE) // checks if new data came
	{
		char C = (char) USART2 -> DR; // reads from USART2 Data Register
		if (C == '+') // extra added for ring buffer showing
		{
			RING_OK = 1;
			return;
		}
		if (C == '\r') // if the interrupt of newline comes
		{
			LINE_OK = 1;
			return;
		}
		uint8_t NEXT = (HEAD + 1) % BUFFER;
		if (NEXT != TAIL) // checks if buffer full
		{
			RING[HEAD] = C;
			HEAD = NEXT;
		}
	}
	if (USART2 -> SR & USART_SR_ORE) // checks if overrun (character miss-out) happened
	{
		(void) USART2 -> DR; // reads value as void for ORE flag clear, missed char not found in this case
	}
}

void USART2_STRING(const char* STRING)
{
	while (*STRING) // loops until pointer is NULL
	{
		while (!(USART2 -> SR & USART_SR_TXE)); // waits until transmit is done
		USART2 -> DR = (uint8_t) (*STRING++); // dereferenced pointer returns char and later sent to DR as byte
	}
}

void DELAY(uint32_t MILLIS)
{
	while (MILLIS != 0) MILLIS--;
}

int MAIN()
{
	PLL_CONFIG(); // Main clock config

	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // clock set for GPIOA
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN; // clock set for USART2
	DELAY(10); // no operation delay for clock initiation

	GPIOA -> MODER &= ~((3UL << 2*2) | (3UL << 3*2)); // clearing MODE Register for PA2 and PA3
	GPIOA -> MODER |= ((2UL << 2*2) | (2UL << 3*2)); // setting the alternate function mode (10b) for PA2 (TX) and PA3(RX)
	GPIOA -> AFR[0] &= ~((0xFUL << 4*2) | (0xFUL << 4*3)); // clearing the alternate function register for PA2 and PA3
	GPIOA -> AFR[0] |= ((7UL << 4*2) | (7UL << 4*3)); // setting 7 in AFR's PA2/3 registers as they are bind to USART TX/RX

	USART2 -> BRR = (24 << 4) | 6;
	/* setting Baud Rate Register:
	 * USARTDIV = (45000000/(16*115200)) = 24.4140625
	 * Mantissa = 24 and
	 * Fraction = .4140625 * 16 = 6[floor] */
	USART2 -> CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // setting USART, Transfer, Receive and RXNE Interrupt (polling done by ISR so CPU is not busy here)

	NVIC_SetPriority(USART2_IRQn, 1); // USART Interrupt Priority set to 1 (manual highest)
	NVIC_EnableIRQ(USART2_IRQn); // Enabling the Interrupt for ISR Handle
	USART2_STRING("LAB 04 (Bare-metal): Ring Buffer -> Type and press Enter:\r\n"); // \r\n for USART newline print for MCs

	while(1)
	{
		if (LINE_OK) // ISR set the LINE_OK flag to 1 when \r arrives
		{
			LINE_OK = 0; // Reset
			char STRING[BUFFER];
			uint8_t ITERATOR = 0;

			while (TAIL != HEAD) // read until the whole thing is extracted
			{
				STRING[ITERATOR++] = RING[TAIL];
				TAIL = (TAIL + 1) % BUFFER; // circulating tail
				if (ITERATOR >= BUFFER - 1) break; // for safety
			}
			STRING[ITERATOR] = '\0'; // null character
			char FINAL[BUFFER + 13]; // for showing in console
			snprintf(FINAL, sizeof(FINAL), "You Typed: %s\r\n", STRING); // concatenating strings checking the overflow
			USART2_STRING(FINAL);
		}
		else if (RING_OK)
		{
			RING_OK = 0;
			char STRING[BUFFER];
			char FINAL[BUFFER + 15]; // for ring buffer show
			uint8_t ITERATOR_RING = 0, ITERATOR_STRING = 0;
			while (ITERATOR_RING < BUFFER - 1 || RING[ITERATOR_RING] != '\0')
			{
				STRING[ITERATOR_STRING++] = RING[ITERATOR_RING++];
			}
			STRING[ITERATOR_STRING] = '\0';
			snprintf(FINAL, sizeof(FINAL), "Ring Buffer: %s\r\n", STRING);
			USART2_STRING(FINAL);
		}
	}
}
