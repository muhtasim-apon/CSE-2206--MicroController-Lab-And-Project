/* Lab 3A - External Interrupt on PC13 ( User Button ) - No HAL
 * PC13 = B1 ( User Button , active LOW )
 * Toggles PA5 (LD2) on each falling edge
 */

#include <stdint.h>
#include <stm32f446xx.h>

void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & (1UL << 13)) { /* Is line 13 pending ? */
		EXTI->PR = (1UL << 13); /* Clear pending (w1c) */
		GPIOA->ODR ^= (1UL << 5); /* Toggle LED */
	}
}

int main(void) {
	/* 1. Enable clocks */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* SYSCFG for EXTI mux */
	__NOP();
	__NOP();

	/* 2. PA5 as push - pull output ( LED) */
	GPIOA->MODER &= ~(3UL << 10);
	GPIOA->MODER |= (1UL << 10);

	/* 3. PC13 as input (reset - default state is already input ) */
	GPIOC->MODER &= ~(3UL << 26); /* explicitly clear to input */

	/* 4. SYSCFG EXTICR4 : route Port C to EXTI line 13
	 * EXTICR [3] controls lines 15:12; EXTI13 -> bits [7:4] = 0010 */
	SYSCFG->EXTICR[3] &= ~(0xFUL << 4);
	SYSCFG->EXTICR[3] |= (0x2UL << 4); /* 0x2 = Port C */

	/* 5. EXTI line 13: falling - edge trigger , unmask */
	EXTI->IMR |= (1UL << 13); /* unmask interrupt */
	EXTI->RTSR &= ~(1UL << 13); /* no rising edge */
	EXTI->FTSR |= (1UL << 13); /* falling edge only */

	/* 6. Enable in NVIC ( priority 1) */
	NVIC_SetPriority(EXTI15_10_IRQn, 1);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	while (1) {
		__WFI(); /* Wait For Interrupt -- sleep until next event */
	}
}
