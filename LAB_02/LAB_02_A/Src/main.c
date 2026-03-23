/* Lab 2A - UART2 Polling TX/RX (No HAL)
 * Board  : STM32F446RE Nucleo-64
 * UART2  : PA2 = TX, PA3 = RX
 * Baud   : 115200, 8N1
 * Clock  : 16 MHz (HSI default)
 */

#include "stm32f446xx.h"

#define SYS_CLK    16000000UL
#define BAUD_RATE  115200UL

void USART2_Init(void)
{
    /* 1. Enable Clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;     // Enable USART2 clock
    __NOP(); __NOP();

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
       USARTDIV = 16000000 / (16 * 115200)
                ≈ 8.6805

       Mantissa = 8
       Fraction = 0.6805 * 16 ≈ 11
    */

    USART2->BRR = (8 << 4) | 11;

    /* 4. Enable USART, TX and RX */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE;
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

int main(void)
{
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
