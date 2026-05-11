
#include"stm32f446xx.h"
#include<stdio.h>
#include<math.h>
#define pie acos(-1)
void Tim3_Init(void)
{
	RCC->APB1ENR |=RCC_APB1ENR_TIM3EN;
	__NOP();
	TIM3->CCR1=0;
	TIM3->CR1=0;//disable time3 counter before configuration
	TIM3->PSC=89;
	TIM3->ARR=999;
	TIM3->CCMR1 &=~(TIM_CCMR1_OC1M);
	//TIM3->CCMR1 |=(TIM_CCMR1_OC1M);
	TIM3->CCMR1 |=(6UL<<TIM_CCMR1_OC1M_Pos);
	TIM3->CCMR1|=TIM_CCMR1_OC1PE;
	TIM3->CCER|=TIM_CCER_CC1E;
	TIM3->CCER&=~(TIM_CCER_CC1P);
	TIM3->CR1|=TIM_CR1_ARPE;
	TIM3->EGR=TIM_EGR_UG;
	TIM3->CR1|=TIM_CR1_CEN;

}
void Tim6_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	__NOP();
	TIM6->CR1=0;
	TIM6->PSC=89;
	TIM6->ARR=0xFFFF;
	TIM6->EGR=TIM_EGR_UG;
	TIM6->SR=0;
	TIM6->CR1 |=TIM_CR1_CEN;
}
void GPIO_INIT(void)
{
	RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN;
		__NOP();//delay for clock stabilization
	GPIOA->MODER &=~(3UL<<12);//alternative function mode
	GPIOA->MODER |=(2UL<<12);
	GPIOA->OTYPER &=~(1UL<<6);//push pull output type
	GPIOA->OSPEEDR &= ~(3UL<<12);
	GPIOA->OSPEEDR |=(3UL<<12);//high speed ensures
	GPIOA->PUPDR &=~(3UL<<12); //No pull up no pull down mode
	GPIOA->AFR[0] &= ~(0xF<<24);//clearing bit for pin 6
	GPIOA->AFR[0] |= (2UL<<24);//value 0010 in 4 bit field of pin 6->enable AF2 MODE

}
//void delay_us(uint32_t us)
//{
//	TIM6->CNT=0;
//	TIM6->SR &=~TIM_SR_UIF;
//	uint32_t overflow=us/TIM6->ARR;
//	uint32_t remainder=us%TIM6->ARR;
//	while(overflow--)
//	{
//		while(!(TIM6->SR & TIM_SR_UIF));
//		TIM6->SR &=~TIM_SR_UIF;
//	}
//	TIM6->CNT=0;
//	while(TIM6->CNT<remainder);
//}
//void delay_us(uint32_t us)
//{
//	uint32_t overflow=us/65535;
//	uint32_t remainder=us%65535;
//	TIM6->CNT=0;
//	TIM6->SR=0;
//	while(overflow--)
//	{
//		while(!(TIM6->SR & TIM_SR_UIF));
//		TIM6->SR=0;
//	}
//	TIM6->CNT=0;
//	while(TIM6->CNT< remainder);
//}
void delay_us(uint32_t us)
{
	uint32_t overflow=us/65536;
	uint32_t remainder=us%65536;
	TIM6->CNT=0;
	TIM6->SR &=~TIM_SR_UIF;
	while(overflow--)
	{
		while(!(TIM6->SR & TIM_SR_UIF));
		TIM6->SR &=~TIM_SR_UIF;
	}
	TIM6->CNT=0;
	while(TIM6->CNT< remainder);
}
void delay_ms(uint32_t ms)
{
	for(int i=0;i<ms;i++)delay_us(1000);
}
void PWM_SetDuty(uint8_t pct)
{
	//uint8_t pct;
	if(pct>100)pct=100;
	uint32_t CCR1 = (uint32_t)(pct * (TIM3->ARR + 1)) / 100;
	TIM3->CCR1=CCR1;
}
void breathing_cycle(void)
		{
			static uint8_t LUT[256];
			//static int initialized = 0;
//			if(!initialized)
//			{
			for(int i=0;i<256;i++)
				{LUT[i]=floor(50.0f*(1.0f+sinf(2.0f*pie*i/256.0f)));

//				}
//			initialized=1;
			}
			//while(1)
			//	{
				for(int i=0;i<256;i++)
			{
				PWM_SetDuty(LUT[i]);
				delay_ms(8);
			}
		}
		//}
void USART2_INIT(void)
{
	RCC->APB1ENR |=RCC_APB1ENR_USART2EN;
	__NOP();
	GPIOA->MODER &=~((3UL <<4 ) | (3UL<<6));
	GPIOA->MODER |=((2UL<<4) |(2UL<<6));
	GPIOA->OTYPER &=~((1UL<<2) | (1UL<<3));
	GPIOA->OSPEEDR &= ~((3UL<<4) | (3UL<<6));
	GPIOA->OSPEEDR |=((3UL<<4) |(3UL<<6));
	GPIOA->PUPDR &=~((3UL<<4) |(3UL<<6));
	GPIOA->AFR[0] &=~((0xF<<8) | (0xF<<12));
	GPIOA->AFR[0] |=((7<<8) |(7<<12));
	USART2->BRR=(24<<4)|6;
	USART2->CR1 |=USART_CR1_UE | USART_CR1_TE| USART_CR1_RE;
}
void PLL_Config(void)
{
	//Enabling fpu unit first
	SCB->CPACR |=((3UL<<10*2)| (3UL<<11*2));//CPACR ER SOFTWARE KE NIYE KAJ KORE
	__DSB();//ETA DOFTWARE DIRECT READ INE ER JONNO
	__ISB();//EKTU INTERRUP O ENABLE KORE DILAM
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
void USART2_SendChar(char c)
{
	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR=(uint8_t)c;
}
void USART2_SendString(char *str)
{
	while(*str)USART2_SendChar(*str++);
}
uint8_t USART2_RecChar(char c)
{
	while(!(USART2->SR & USART_SR_RXNE));
	return (uint8_t)(USART2->DR);
}
int main(void)
{
	PLL_Config();
	GPIO_INIT();
	Tim6_Init();
	Tim3_Init();
	USART2_INIT();
	char msg[50];
	USART2_SendString("Duty Cycle and CRR Value Record:\r\n");
	for(int i=0;i<=100;i+=10)
	{
		PWM_SetDuty(i);
	sprintf(msg,"Duty = %d%% CCR1 = %lu\r\n",i,TIM3->CCR1);
	USART2_SendString(msg);
	delay_ms(300);
	}
	USART2_SendString("Five Complete Breathing cycle\r\n");
	for(int i=0;i<5;i++)
	{
		char msg[30];
		sprintf(msg, "Cycle = %d \r\n",i+1);
		USART2_SendString(msg);
		breathing_cycle();
	}
	PWM_SetDuty(50);
			sprintf(msg,"at Exactly Duty = 50%% CCR1 = %lu\r\n",TIM3->CCR1);
			//PWM_SetDuty(50);
			USART2_SendString(msg);
		//breathing_cycle();//wait for 2 seconds

//	while(1)
//	{
//	}

	}
