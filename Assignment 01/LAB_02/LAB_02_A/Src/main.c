/* Lab 2A - UART2 Polling TX/RX (No HAL)
 * Board  : STM32F446RE Nucleo-64
 * UART2  : PA2 = TX, PA3 = RX
 * Baud   : 115200, 8N1
 * Clock  : 16 MHz (HSI default)
 */

#include "stm32f446xx.h"

#define APB1_CLK   45000000UL
#define BAUD_RATE  115200UL

void USART2_Init(void)
{
    /* 1. Enable Clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;     // Enable USART2 clock
    __NOP();
    __NOP();

    /* 2. Configure PA2 and PA3 as Alternate Function AF7 (USART2) */
    /* Alternate Function Mode */
    GPIOA->MODER &= ~((3UL<<(2*2)) | (3UL<<(3*2)));
    GPIOA->MODER |=  ((2UL<<(2*2)) | (2UL<<(3*2)));

    /* Select AF7 */
    GPIOA->AFR[0] &= ~((0xF<<(4*2)) | (0xF<<(4*3)));
    GPIOA->AFR[0] |=  ((7<<(4*2))  | (7<<(4*3)));

    /* Push-pull */
    GPIOA->OTYPER &= ~((1UL<<2) | (1UL<<3));

    /* Very high speed */
    GPIOA->OSPEEDR |= ((3UL<<(2*2)) | (3UL<<(3*2)));

    /* No pull-up / pull-down */
    GPIOA->PUPDR &= ~((3UL<<(2*2)) | (3UL<<(3*2)));

    /* 3. Configure Baud Rate */
    /*
       USARTDIV = 45000000 / (16 * 115200)
                ≈ 24.4140625

       Mantissa = 24
       Fraction = 0.4140625 * 16 ≈ 6
    */
    USART2->BRR = (24 << 4) | 6;

    /* 4. Enable USART, TX and RX */
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART2->CR1 |= USART_CR1_UE;
}

void USART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));   // Wait for TX buffer empty
    USART2->DR = (uint8_t)c;
}

void USART2_SendString(const char *str)
{
    while (*str)
    {
        USART2_SendChar(*str++);
    }
}

char USART2_RecvChar(void)
{
    while (!(USART2->SR & USART_SR_RXNE));  // Wait until data received
    return (char)(USART2->DR);
}

void PLL_Config(void)
{
    /* 1. Enable HSI (16 MHz) */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));  // Wait until HSI ready

    /* 2. Configure PLL: HSI/16 * 180 / 2 = 90 MHz */
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos)   // PLLM = 16
                 | (180 << RCC_PLLCFGR_PLLN_Pos)  // PLLN = 180
                 | (0 << RCC_PLLCFGR_PLLP_Pos)    // PLLP = 2 (00b)
                 | (4 << RCC_PLLCFGR_PLLQ_Pos)    // PLLQ = 4 (USB/SDIO)
                 | RCC_PLLCFGR_PLLSRC_HSI;        // Source = HSI

    /* 3. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait until PLL ready

    /* 4. Configure Flash latency (2 WS for 90 MHz) */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;

    /* 5. Set prescalers: AHB=1, APB1=2, APB2=1 */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV2
              | RCC_CFGR_PPRE2_DIV1;

    /* 6. Switch SYSCLK to PLL */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait until PLL is active
}

int main(void)
{
	PLL_Config();
    USART2_Init();
    USART2_SendString("STM32F446RE UART Polling Demo\r\n");
    USART2_SendString("Type a character -- it will be echoed:\r\n");

    while (1)
    {
        char c = USART2_RecvChar();   // Receive character
        USART2_SendChar(c);           // Echo back

        if (c == '\r')
        {
            USART2_SendChar('\n');
        }
    }
}
