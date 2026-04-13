/* Lab 5A - UART2 DMA TX (Stream6 Ch4) + Circular RX (Stream5 Ch4) [Comment out]
 * No HAL -- direct DMA register configuration
 */
// Following code is for DMA RX, similar as in HAL code

#include <stdint.h>
#include <stm32f446xx.h>
#include <string.h>

#define RX_BUF_SIZE 64
static char txMsg[] = "STM32 No-HAL DMA UART Transmit Demo\r\n";
static volatile char rxBuf[RX_BUF_SIZE];

#define APB1_CLK 45000000UL
#define BAUD_RATE 115200UL

void SystemClock_Config(void)
{
    /* 1. Enable PWR clock */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    /* 2. Voltage scaling (Scale 1 mode) */
    PWR->CR |= PWR_CR_VOS;

    /* 3. Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    while(!(RCC->CR & RCC_CR_HSIRDY));

    /* 4. Configure FLASH latency and enable caches */
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_5WS;

    /* 5. Configure PLL
       HSI = 16 MHz
       PLLM = 8
       PLLN = 180
       PLLP = 2
       PLLQ = 2
    */

    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (180 << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLP_Pos);   // PLLP = 2
    RCC->PLLCFGR |= (RCC_PLLCFGR_PLLSRC_HSI);
    RCC->PLLCFGR |= (2 << RCC_PLLCFGR_PLLQ_Pos);

    /* 6. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY));

    /* 7. Enable OverDrive mode */
    PWR->CR |= PWR_CR_ODEN;
    while(!(PWR->CSR & PWR_CSR_ODRDY));

    PWR->CR |= PWR_CR_ODSWEN;
    while(!(PWR->CSR & PWR_CSR_ODSWRDY));

    /* 8. Configure Bus Prescalers
       AHB = SYSCLK /1
       APB1 = HCLK /4
       APB2 = HCLK /2
    */

    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* 9. Select PLL as system clock */
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void DMA1_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	__NOP();
	__NOP();

	/* --- TX: DMA1 Stream6 Channel4 ( Memory -> Peripheral ) --- */
	DMA1_Stream6->CR = 0; /* disable first */

	while (DMA1_Stream6->CR & DMA_SxCR_EN); /* wait disabled */

	DMA1_Stream6->PAR = (uint32_t) &USART2->DR;
	DMA1_Stream6->CR = (4UL << DMA_SxCR_CHSEL_Pos) /* Channel 4 */
	| DMA_SxCR_DIR_0 /* Memory -> Peripheral */
	| DMA_SxCR_MINC /* Memory increment */
	| DMA_SxCR_TCIE; /* TC interrupt */

	/* --- RX: DMA1 Stream5 Channel4 ( Peripheral -> Memory , Circular ) --- */
	DMA1_Stream5->CR = 0;

	while ( DMA1_Stream5->CR & DMA_SxCR_EN);

	DMA1_Stream5->PAR = (uint32_t) &USART2->DR;
	DMA1_Stream5->M0AR = (uint32_t) rxBuf;
	DMA1_Stream5->NDTR = RX_BUF_SIZE;

	// LAB manual for Circular RX:
	// DMA1_Stream5->CR = (4UL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_CIRC /* Circular mode */
	//											   | DMA_SxCR_MINC;

	// Code for DMA RX:
	DMA1_Stream5->CR = (4UL << DMA_SxCR_CHSEL_Pos) |
	                   DMA_SxCR_MINC |
	                   DMA_SxCR_TCIE;   // interrupt when 64 bytes received

	DMA1_Stream5->CR |= DMA_SxCR_EN; /* Enable RX DMA */

	NVIC_SetPriority(DMA1_Stream6_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Stream6_IRQn);

	// RX DMA interrupt
	NVIC_SetPriority(DMA1_Stream5_IRQn, 1);
	NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/* TX Transfer Complete ISR */
void DMA1_Stream6_IRQHandler(void) {
	if (DMA1->HISR & DMA_HISR_TCIF6) {
		DMA1->HIFCR = DMA_HIFCR_CTCIF6; /* clear TC flag */

		USART2->CR3 &= ~ USART_CR3_DMAT; /* disable UART DMA TX */

		DMA1_Stream6->CR &= ~ DMA_SxCR_EN;
	}
}

void DMA_UART_Transmit(const char *data, uint16_t len) {
	while (DMA1_Stream6->CR & DMA_SxCR_EN); /* wait if busy */

	DMA1_Stream6->M0AR = (uint32_t) data;
	DMA1_Stream6->NDTR = len;

	USART2->CR3 |= USART_CR3_DMAT; /* enable UART DMA TX */

	DMA1_Stream6->CR |= DMA_SxCR_EN; /* start transfer */
}

// RX Receive complete ISR: for DMA RX. Not needed for circular RX
void DMA1_Stream5_IRQHandler(void)
{
    if (DMA1->HISR & DMA_HISR_TCIF5)
    {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;   // clear flag

        /* Echo received 64 bytes */
        DMA_UART_Transmit((char*)rxBuf, RX_BUF_SIZE);

        /* Restart RX DMA */
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream5->CR & DMA_SxCR_EN);

        DMA1_Stream5->NDTR = RX_BUF_SIZE;
        DMA1_Stream5->CR |= DMA_SxCR_EN;
    }
}

void USART2_DMA_Init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	__NOP();
	__NOP();

	GPIOA->MODER &= ~((3UL << 4) | (3UL << 6));
	GPIOA->MODER |= ((2UL << 4) | (2UL << 6));
	GPIOA->AFR[0] &= ~((0xFUL << 8) | (0xFUL << 12));
	GPIOA->AFR[0] |= ((7UL << 8) | (7UL << 12));

	USART2->BRR = (24 << 4) | 7;   // 0x187
	USART2->CR3 |= USART_CR3_DMAR; /* enable UART DMA RX */
	USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

int main(void) {
	SystemClock_Config();
	USART2_DMA_Init();
	DMA1_Init();
	DMA_UART_Transmit(txMsg, strlen(txMsg)); /* non - blocking TX */

	// Commented code are from Lab Manual:
	// uint16_t prevNDTR = RX_BUF_SIZE;
	while (1) {
		/* Detect new RX data by monitoring NDTR decrement */
		// uint16_t curNDTR = DMA1_Stream5->NDTR;
		// if (curNDTR != prevNDTR) {
			  /* New data in rxBuf -- process here */
		//	  prevNDTR = curNDTR;
		// }
		__WFI();
	}
}
