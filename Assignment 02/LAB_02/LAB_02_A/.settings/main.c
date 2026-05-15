
#include "stm32f446xx.h"
#include<stdio.h>

void USART2_Init(void)
{
    /* 1. Enable Clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;      // Enable GPIOA clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;     // Enable USART2 clock
    __NOP();
    __NOP();

    /* 2. Configure PA2 and PA3 as Alternate Function AF7 (USART2) */
    /* Alternate Function Mode */
    GPIOA->MODER &= ~((3UL<<(2*2)) | (3UL<<(3*2)));//pa2 and pa3 clear
    GPIOA->MODER |=  ((2UL<<(2*2)) | (2UL<<(3*2))); //now pa2 and pa3 alternate mode

    /* Select AF7 */

    GPIOA->AFR[0] &= ~((0xF<<(4*2)) | (0xF<<(4*3))); //first clear
    GPIOA->AFR[0] |=  ((7<<(4*2))  | (7<<(4*3))); //then af7 mode --> for the usart mode

    /* Push-pull */
    GPIOA->OTYPER &= ~((1UL<<2) | (1UL<<3)); //push pull

    /* Very high speed */
    GPIOA->OSPEEDR |= ((3UL<<(2*2)) | (3UL<<(3*2))); //very high speed

    /* No pull-up / pull-down */
    GPIOA->PUPDR &= ~((3UL<<(2*2)) | (3UL<<(3*2))); //no pull up no pull down

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

void PA5_INIT(void)
{
	GPIOA->MODER &= ~(3UL<<10);
	GPIOA->MODER |=(1UL<<10);
	GPIOA->OTYPER &= ~(1UL<<5);
	GPIOA->OSPEEDR &=~(3UL<<10);
	GPIOA->PUPDR &=~(3UL<<10);
}
void PLL_Config(void)
{
    /* 1. Enable HSI (16 MHz) */
    RCC->CR |= RCC_CR_HSION;//enabling hsi
    while (!(RCC->CR & RCC_CR_HSIRDY));  // Wait until HSI ready,confirming hsi is ready ornot

    /* 2. Configure PLL: HSI/16 * 180 / 2 = 90 MHz */
    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos)   // PLLM = 16 -->into 1Mhz convert
                 | (180 << RCC_PLLCFGR_PLLN_Pos)  // PLLN = 180 --> now its 180MHz. this is actual pll clk source
                 | (0 << RCC_PLLCFGR_PLLP_Pos)    // PLLP = 2 (00b) --> system clk --> 90MHz
                 | (4 << RCC_PLLCFGR_PLLQ_Pos)    // PLLQ = 4 (USB/SDIO)
                 | RCC_PLLCFGR_PLLSRC_HSI;        // Source = HSI

    /* 3. Enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait until PLL ready,pll configure till wait

    /* 4. Configure Flash latency (2 WS for 90 MHz) */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS; //90/30=(3-1)=2 coz 1 wait cycle to match speed

    /* 5. Set prescalers: AHB=1, APB1=2, APB2=1 */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 //90/1
              | RCC_CFGR_PPRE1_DIV2 //90/2
              | RCC_CFGR_PPRE2_DIV1; //90/1

    /* 6. Switch SYSCLK to PLL */
    RCC->CFGR |= RCC_CFGR_SW_PLL; //switching to pll , for the clk source now .
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait until PLL is active
}

void delay_us(uint32_t us)
{
	uint32_t overflow=us/65536; //splitting the delay
	uint32_t remainder=us%65536; //rest ticks or microseconds
	TIM6->CNT=0; //reset counter
	TIM6->SR=~TIM_SR_UIF; //clear overflow flag
	while(overflow--)
	{
		while(!(TIM6->SR & TIM_SR_UIF)); //0 ->arr full time cycle  -->65.536 micro seconds
		TIM6->SR=~TIM_SR_UIF; //clearing the uif
	}
	TIM6->CNT=0; //safety issue
	while(TIM6->CNT< remainder); //extra counts
}
void delay_ms(uint32_t ms)
{
	for(int i=1;i<=ms;i++)delay_us(1000);

}
//eta mainly 16 bit counter register, jodi ekhane loop chalai per iteration e 1000 us lagbe . but oitare gun dile seta 16 bit cross korar chance thke . eikarone eta robust mainly

void delay_s(uint32_t sec)
{
	for(int i=1;i<=sec;i++)delay_ms(1000);

}

void delay_hms(uint8_t h, uint8_t m , uint8_t s)
{
	uint32_t total_s=(uint32_t)h*3600 +(uint32_t) m*60 +s;
	delay_s(total_s);
}

void USART2_SendChar(char c)
{
	while(!(USART2->SR & USART_SR_TXE))
	{
		//while txe is empty it will wait
	}
		USART2->DR=(uint8_t)c;
}
void USART2_SendString(char *str)
{
	while (*str) USART2_SendChar(*str++);
}
uint8_t USART2_RevChar(void)
{
	while(!(USART2->SR & USART_SR_RXNE));
	return (uint8_t)(USART2->DR & 0XFF);
}
void Tim_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;//enabling timer 6
		__NOP(); //stabilising for the clock enable
		__NOP();
		TIM6->CR1 &= ~TIM_CR1_CEN; // count start from 0, disabling timer
		TIM6->PSC=89; //90/(89+1)=1MHz
		TIM6->ARR=~0x0; //then arr=65535 highest value
		TIM6->EGR |=TIM_EGR_UG; //for update event, apply the psc/arr immediately that are buffered.
		TIM6->SR=0; //status flag -->uif , false overflow detection
		TIM6->CR1 |=TIM_CR1_CEN; // now starts timer , enable counting
}
int main(void)
{
	PLL_Config();
	USART2_Init();
	Tim_Init();
	PA5_INIT();
	char msg[64];
	//while(1)
	//{
		//uint32_t test_ms=2000;
		//sprintf(msg,"Starting %lu ms delay..\r\n", test_ms);
		//USART2_SendString(msg);
		USART2_SendString("1.Starting 500 ms Delay:\r\n");
		delay_ms(500);
		USART2_SendString("Completed 500 ms Delay:\r\n");
		USART2_SendString("2.Starting 1000 ms Delay:\r\n");
		delay_ms(1000);
		USART2_SendString("Completed 1000 ms Delay:\r\n");
		//delay_ms(test_ms);
		int j=1;
		for(int i=1;i<=5;i++)
		{
			sprintf(msg,"3.Toggle %d Status :On\r\n",j++);
			USART2_SendString(msg);
			GPIOA->ODR ^=(1UL<<5);
			delay_ms(250);
			sprintf(msg,"3.Toggle %d Status :Off\r\n",j++);
			USART2_SendString(msg);
			GPIOA->ODR ^=(1UL<<5);
			delay_ms(250);
		}
		USART2_SendString("4.Starting 3 sec Delay:\r\n");
		delay_s(3);
		USART2_SendString("4.Completed 3 sec Delay:\r\n");
		USART2_SendString("5.Starting 0 H 0 M 5 sec Delay:\r\n");
		delay_hms(0,0,5);
		USART2_SendString("5.Completed 0 H 0 M 5 sec Delay:\r\n");
		//USART2_SendString("Done.\r\n");
		//delay_s(1);
	//}

}
