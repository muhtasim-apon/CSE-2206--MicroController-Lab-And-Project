#include "stm32f446xx.h"

void delay_ms(uint32_t ms){
	for(uint32_t i=0;i<ms*4000;i++)
	{
		__NOP();
	}
}

int main(void){
	RCC->AHB1ENR |=RCC_AHB1ENR_GPIOAEN;
	__NOP(); __NOP();

	GPIOA->MODER &= ~(3UL << (5*2));
	GPIOA->MODER |= (1UL << (5*2));

	GPIOA->OTYPER &= ~(1UL <<5);

	GPIOA->OSPEEDR &= ~(3UL << (5*2));

	GPIOA->PUPDR &= ~(3UL << (5*2));

	while(1){
		GPIOA->BSRR= (1UL <<5);
		delay_ms(100);
		GPIOA->BSRR= (1UL << (5+16));
		delay_ms(100);
	}

}
