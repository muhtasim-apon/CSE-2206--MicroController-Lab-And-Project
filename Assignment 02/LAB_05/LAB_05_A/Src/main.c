<<<<<<< HEAD
#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define WS_ARR      224U
#define WS_T1H      144U
#define WS_T0H       72U
#define WS_RESET     50U
#define NUM_LEDS      5U
#define PWM_BUF_SIZE  ((NUM_LEDS * 24U) + WS_RESET)

#define SEQ_VISIBLE_DELAY_MS  100U
#define SEQ_END_OFF_DELAY_MS  2000U
#define PALETTE_COUNT (sizeof(palette) / sizeof(palette[0]))

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} LED_t;

typedef struct
{
    const char *name;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} Colour_t;

static const Colour_t palette[] =
{
    {"Red",        255,   0,   0},
    {"Green",        0, 255,   0},
    {"Blue",         0,   0, 255},
    {"Yellow",     255, 255,   0},
    {"Cyan",         0, 255, 255},
    {"Magenta",    255,   0, 255},
    {"White",      255, 255, 255},
    {"Warm White", 255, 200,  80},
    {"DU Blue",     31,  56, 100},
    {"Off",          0,   0,   0}
};

LED_t g_leds[NUM_LEDS];
volatile uint8_t datasentflag = 0;
uint16_t pwmData[PWM_BUF_SIZE];

void PLL_CONFIG(void)
{
	// Power-Control Peripheral enable
    RCC -> APB1ENR |= RCC_APB1ENR_PWREN;
    // Set voltage regulator scaling for high-speed clock operation for PLL
    PWR -> CR |= PWR_CR_VOS;

    // Configure Flash Latency for PLL_CLK = 180MHz at Access Control Register
	FLASH -> ACR =
			FLASH_ACR_ICEN | // Instruction Cache Enable
			FLASH_ACR_DCEN | // Data Cache Enable
			FLASH_ACR_PRFTEN | // Flash Pre-fetch Enable for Faster Operation
			FLASH_ACR_LATENCY_5WS; // Latency of 5 Wait States as per state cover approximately 30MHz

    // Enable Over-Drive Mode for higher clocks
    PWR -> CR |= PWR_CR_ODEN;
    // Wait until ready
	while (!(PWR->CSR & PWR_CSR_ODRDY));

	// Enabling Over-Drive Switch Mode for switching when needed
	PWR -> CR |= PWR_CR_ODSWEN;
	// Wait until ready
	while (!(PWR->CSR & PWR_CSR_ODSWRDY));

    // initialize the External High-Speed Oscillator as the Clock for PLL (Phase-Locked Loop), external clock works better in case of latency
	RCC -> CR |= RCC_CR_HSEON;
	// wait until HSE is initialized
	while (!(RCC -> CR & RCC_CR_HSERDY));

	// Configuration of PLL
	RCC -> PLLCFGR =
			(8 << RCC_PLLCFGR_PLLM_Pos) | // 8MHz HSE -> 1MHz (Pre-scaler)
			(360 << RCC_PLLCFGR_PLLN_Pos) | // 1MHz to 360MHz (Multiplier) -> PLL VCO Clock
			(0 << RCC_PLLCFGR_PLLP_Pos) | // 360 to 180MHz -> PLL_CLK or SYSCLK
			(2 << RCC_PLLCFGR_PLLQ_Pos) | // VCO / 2 -> CLK for USB/SDIO (Not used here)
			RCC_PLLCFGR_PLLSRC_HSE; // Base Clock Source -> HSE

	// initialize the PLL for operation
	RCC -> CR |= RCC_CR_PLLON;
	while (!(RCC -> CR & RCC_CR_PLLRDY));

    // Configure pre-scalers
    RCC -> CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC -> CFGR |=
			RCC_CFGR_HPRE_DIV1 | // PLL_CLK -> AHB Bus -> 180MHz
			RCC_CFGR_PPRE1_DIV4 | // PLL_CLK -> APB1 Bus -> 45MHz
			RCC_CFGR_PPRE2_DIV2; // PLL_CLK -> APB2 Bus -> 90MHz

    // Switch SYSCLK to PLL
    RCC -> CFGR &= ~RCC_CFGR_SW;
    RCC -> CFGR |= RCC_CFGR_SW_PLL;
    // Wait until PLL is used as system clock
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void GPIOA_CONFIG(void)
{
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // clock for GPIO Ports

	/* PA8 as the PWM Output Pin */
	GPIOA -> MODER &= ~(3UL << 8*2); // PA8 clear
	GPIOA -> MODER |= (2UL << 8*2); // PA8 mode -> Alternate Function

	GPIOA -> OTYPER &= ~(1UL << 8); // PA8 clear leads to Push-Pull mode

	GPIOA -> OSPEEDR &= ~(3UL << 8*2); // PA8 clear
	GPIOA -> OSPEEDR |= (3UL << 8*2); // PA8 set to very-high speed

	GPIOA -> PUPDR &= ~(3UL << 8*2); // PA8 to no pull mode

	GPIOA -> AFR[1] &= ~(0xF << (8-8)*4); // PA8 clear in AFR High, 8-8 because PA8 lies in [3:0]
	GPIOA -> AFR[1] |= (0x1 << (8-8)*4); // PA8 set to AF1

	/* PA2/3 as USART2 TX/RX */
	GPIOA -> MODER &= ~((3UL << 2*2) | (3UL << 3*2)); // Reset MODE Register of GPIOA of PA2, PA3
	GPIOA -> MODER |= ((2UL << 2*2) | (2UL << 3*2)); // Set MODE Register to Alternate-Function Mode for PA2(TX), PA3(RX)

	GPIOA -> AFR[0] &= ~((0xF << 2*4) | (0xF << 3*4)); // Clearing the AFR Register Low for PA2, PA3 (each take 4 bits)
	GPIOA -> AFR[0] |= ((7UL << 2*4) | (7UL << 3*4)); // Set the AFR Register Low in PA2, PA3 Position to USART2 TX/RX

	GPIOA -> OTYPER &= ~((1UL << 2) | (1UL << 3)); // Set output type to push-pull -> 0b0
	GPIOA -> OSPEEDR |= ((3UL << 2*2) | (3UL << 3*2)); // Set Output Speed at very high -> 0b11
	GPIOA -> PUPDR &= ~((3UL << 2*2) | (3UL << 3*2)); // Set pull-up pull down to no pull-up no pull-down -> 0b00
}

void USART2_CONFIG(void)
{
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN; // clock sent to USART2 Ports
	__NOP(); // delay for clock initiation

	/*
	USARTDIV = 45000000 / (16 * 115200)
			 ≈ 24.4140625
	Mantissa = 24
	Fraction = 0.4140625 * 16 ≈ 7
	*/
	USART2 -> BRR = (24 << 4) | 7;

	// Enable USART2
    USART2 -> CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART2_SendString(const char *s)
{
    while (*s)
    {
        while (!(USART2 -> SR & USART_SR_TXE));
        USART2 -> DR = (uint8_t)(*s++);
    }
    while (!(USART2->SR & USART_SR_TC));
}

void TIM6_CONFIG(void)
{
    RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable TIM6 for delay count
    __NOP(); __NOP(); // Wait until ready

    TIM6 -> CR1 &= ~TIM_CR1_CEN; // Disable Counter for safety
    TIM6 -> PSC = 89U; // Pre-scaler to feed onto clock. f(TIM6) = 90/(89+1) MHz = 1MHz, per tick = 1us
    TIM6 -> ARR = 0xFFFFU; // Auto Reload-Register to Free Count Mode, max value set
    TIM6 -> EGR = TIM_EGR_UG; // Update PSC and ARR onto shadow registers immediately
    TIM6 -> SR = 0U; // Status Flags set to 0
    TIM6 -> CR1 |= TIM_CR1_CEN; // Enable Counter
}

void delay_us(uint16_t us)
{
    TIM6 -> CNT = 0U;
    while ((uint16_t)TIM6 -> CNT < us);
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        delay_us(1000U);
    }
}

void TIM1_WS_CONFIG(void)
{
    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN; // Clock for TIM1

    TIM1 -> PSC = 0U; // No Pre-scaler, gets 180MHz from the APB2 (90*2 = 180MHz)
    TIM1 -> ARR = WS_ARR; // ARR = 224 so ARR + 1 = 225
    TIM1 -> CCR1 = 0U; // Initially set Capture - Compare Value to 0

    TIM1 -> CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; // Set output compare mode to 0b110 (PWM Mode 1), with Pre-load Enabled
    TIM1 -> CCER = TIM_CCER_CC1E; // Enable Capture Compare Mode
    TIM1 -> BDTR = TIM_BDTR_MOE; // Main Output Enabled, so output can be showed onto PA8 Pin (must for TIM1/8)
    TIM1 -> EGR = TIM_EGR_UG; // Set PSC and ARR values immediately onto shadow registers
    TIM1 -> SR = 0U; // Disable all flags
}

void DMA2_WS_CONFIG(void)
{
    RCC -> AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enabling Clocks

    DMA2_Stream1 -> CR &= ~DMA_SxCR_EN; // Disabling Stream1_ControlRegister_Enable flag -> Disable Stream
    while (DMA2_Stream1 -> CR & DMA_SxCR_EN); // Wait until ready disabling

    DMA2 -> LIFCR = // Sets Low Interrupt Flag Clear Registers
    		DMA_LIFCR_CFEIF1  | // FIFO Error Interrupt Flag
			DMA_LIFCR_CDMEIF1 | // Direct Mode Error Interrupt Flag
			DMA_LIFCR_CTEIF1  | // Transfer Error Interrupt Flag
			DMA_LIFCR_CHTIF1  | // Half Transfer Interrupt Flag
			DMA_LIFCR_CTCIF1; // Transfer Complete Interrupt Flag

    DMA2_Stream1 -> CR = // Stream1 Configuration
    		(6U << 25U) | // Set DMA Stream1 to Channel 6 [:25]
			(2U << 16U) | // Priority Level = High [17:16], because timing is strict
			(1U << 13U) | // Memory Data Size = Half-Word (as pwmData uses uint16_t)
			(1U << 11U) | // Peripheral Data Size = Half-Word (DMA set CCR1 values, as TIM1 is 16 bit, Half-Word preferred)
			(1U << 10U) | // Enable Memory Increment so that pwmData[i] can be called
			(1U << 6U)  | // Transfer Direction from Memory to Peripheral as pwmData(Memory) -> CCR1(Peripheral)
			(1U << 4U); // Enable Transfer Complete Interrupt for calling dedicated Handler

    DMA2_Stream1 -> PAR = (uint32_t)&TIM1 -> CCR1; // Setting Peripheral Address of DMA to the CCR1 Register's Address
    DMA2_Stream1 -> FCR &= ~DMA_SxFCR_DMDIS; // FIFO Control Register clears Direct Mode Disable Bit, so mode is selected as Direct (pwmData -> CCR1, no buffer between)

    NVIC_SetPriority(DMA2_Stream1_IRQn, 0U); // Set Interrupt Priority
    NVIC_EnableIRQ(DMA2_Stream1_IRQn); // Enable Interrupt
}

void DMA2_Stream1_IRQHandler(void)
{
    if (DMA2 -> LISR & DMA_LISR_TCIF1) // Checks if Interrupt triggered via transfer complete in Low Interrupt Status Register
    {
        DMA2 -> LIFCR = DMA_LIFCR_CTCIF1; // Clear the flag
        TIM1 -> CR1 &= ~TIM_CR1_CEN; // Disable TIM1 Counter
        DMA2_Stream1 -> CR &= ~DMA_SxCR_EN; // Disable DMA
        TIM1 -> DIER &= ~TIM_DIER_CC1DE; // Disables DMA/Interrupt of Capture Compare 1
        datasentflag = 1; // Flag Set for data send inquiry
    }
}

void WS2812B_SEND(void)
{
    uint32_t idx = 0;

    for (uint32_t led = 0; led < NUM_LEDS; led++) // Data Stream for each LED
    {
        uint32_t color = // Set color in GRB order (Expected for WS2812)
            ((uint32_t)g_leds[led].g << 16) |
            ((uint32_t)g_leds[led].r << 8)  |
            ((uint32_t)g_leds[led].b);

        for (int bit = 23; bit >= 0; bit--) // Sending 24 bits of data to pwmData buffer
            pwmData[idx++] = (color & (1U << bit)) ? WS_T1H : WS_T0H; // each color bit sets -> 0 = T0H, 1 = T1H
    }

    for (uint32_t i = 0; i < WS_RESET; i++) pwmData[idx++] = 0U; // Reset latch of 50 iterations

    TIM1 -> CR1 &= ~TIM_CR1_CEN; // Counter Disable

    DMA2_Stream1 -> CR &= ~DMA_SxCR_EN; // DMA Disable
    while (DMA2_Stream1->CR & DMA_SxCR_EN); // Wait Until Disabling DMA is finished

    DMA2 -> LIFCR = 0x3D << 0; // Setting the Low Interrupt Clear Flags

    DMA2_Stream1 -> M0AR = (uint32_t)pwmData; // Setting the Memory 0 Address Register as pwmData's address
    DMA2_Stream1 -> NDTR = idx; // Setting number of bits to be sent by DMA

    /* Timers Reset */
    TIM1 -> CCR1 = 0U;
    TIM1 -> CNT = 0U;
    TIM1 -> SR = 0U;

    TIM1 -> DIER |= TIM_DIER_CC1DE; // Enable Capture-Compare 1 for DMA/Interrupt
    DMA2_Stream1 -> CR |= DMA_SxCR_EN; // Enable DMA Stream 1
    TIM1 -> CR1 |= TIM_CR1_CEN; // Enable Counter

    while (!datasentflag); // Wait until DMA triggers an interrupt of Transfer Complete
    datasentflag = 0; // Reset for next WS2812_SEND()
}

void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b) // Setting r,g,b value explicitly
{
    for (uint32_t i = 0; i < NUM_LEDS; i++)
    {
        g_leds[i].r = r;
        g_leds[i].g = g;
        g_leds[i].b = b;
    }
    WS2812B_SEND();
}

void WS2812_ClearBuffer(void) // Clearing pwmData buffer
{
    for (uint32_t i = 0; i < NUM_LEDS; i++)
    {
        g_leds[i].r = 0;
        g_leds[i].g = 0;
        g_leds[i].b = 0;
    }
}

void WS2812_ShowOneLed(uint8_t led_index, uint8_t r, uint8_t g, uint8_t b) // For showing light into one LED only
{
    WS2812_ClearBuffer(); // Clears all LED buffers
    if (led_index < NUM_LEDS) // safe check
    {
        g_leds[led_index].r = r;
        g_leds[led_index].g = g;
        g_leds[led_index].b = b;
    }
    WS2812B_SEND();
}

void WS2812_SetHue(uint16_t H)
{
    H %= 360U;

    uint8_t seg = H / 60U;
    uint8_t frac = H % 60U;

    uint8_t q = 255U * (60U - frac) / 60U; // q -> decreases high to low (falling brg)
    uint8_t t = 255U * frac / 60U; // t -> increases low to high (rising brg)

    uint8_t r, g, b;

    switch (seg)
    {
        case 0:
            r = 255;
            g = t;
            b = 0;
            break;

        case 1:
            r = q;
            g = 255;
            b = 0;
            break;

        case 2:
            r = 0;
            g = 255;
            b = t;
            break;

        case 3:
            r = 0;
            g = q;
            b = 255;
            break;

        case 4:
            r = t;
            g = 0;
            b = 255;
            break;

        case 5:
            r = 255;
            g = 0;
            b = q;
            break;

        default:
            r = 0;
            g = 0;
            b = 0;
            break;
    }

    WS2812_SetAll(r, g, b);
}

void Run_46_Combination_Led_Sequence(void)
{
    char buf[180];
    uint32_t sequence_number = 1;

    USART2_SendString("\r\n[Task 02] 46 Step LED Sequence\r\n");
    USART2_SendString(" n |    LED1    |    LED2    |    LED3    |    LED4    |    LED5   \r\n");

    /*
       First 9 colors only.
       Off is not included here.
       9 colors x 5 LEDs = 45 steps.
    */
    for (uint32_t color = 0; color < 9; color++)
    {
        for (uint32_t led = 0; led < NUM_LEDS; led++)
        {
            WS2812_ShowOneLed(
                led,
                palette[color].r,
                palette[color].g,
                palette[color].b
            );

            snprintf(
                buf,
                sizeof(buf),
                "%-2lu | %-10s | %-10s | %-10s | %-10s | %-10s\r\n",
                (unsigned long)sequence_number,
                (led == 0) ? palette[color].name : "0",
                (led == 1) ? palette[color].name : "0",
                (led == 2) ? palette[color].name : "0",
                (led == 3) ? palette[color].name : "0",
                (led == 4) ? palette[color].name : "0"
            );

            USART2_SendString(buf);
            sequence_number++;
            delay_ms(SEQ_VISIBLE_DELAY_MS);
        }
    }

    WS2812_SetAll(0, 0, 0);
    snprintf(
        buf,
        sizeof(buf),
        "%lu | ---------- | ---------- | ---------- | ---------- | ---------- \r\n",
        (unsigned long)sequence_number
    );

    USART2_SendString(buf);
    delay_ms(SEQ_END_OFF_DELAY_MS);

    USART2_SendString("46 step sequence complete. Final OFF delay = 2s\r\n");
}

int main(void)
{
    char buf[128];
    PLL_CONFIG();
    GPIOA_CONFIG();
    USART2_CONFIG();
    TIM6_CONFIG();
    TIM1_WS_CONFIG();
    DMA2_WS_CONFIG();

    USART2_SendString("LAB 04 (Bare-Metal): WS2812B LED Strip Lighting\r\n");
    WS2812_SetAll(0, 0, 0);
    delay_ms(100);

    USART2_SendString("[Task 01] Color palette - 1s per color\r\n");

    for (uint32_t i = 0; i < PALETTE_COUNT; i++)
    {
        WS2812_SetAll(palette[i].r, palette[i].g, palette[i].b);

        snprintf(
            buf,
            sizeof(buf),
            "Colour: %-12s R=%3u G=%3u B=%3u GRB=[%02X %02X %02X]\r\n",
            palette[i].name,
            palette[i].r,
            palette[i].g,
            palette[i].b,
            palette[i].g,
            palette[i].r,
            palette[i].b
        );

        USART2_SendString(buf);
        delay_ms(1000);
    }

    Run_46_Combination_Led_Sequence();
    USART2_SendString("\r\n[Task 03] Hue sweep 0-359 for 25 ms\r\n");

    while (1)
    {
        for (uint16_t h = 0; h < 360; h += 3)
        {
            WS2812_SetHue(h);
            delay_ms(25);
        }
        USART2_SendString("Hue sweep repeat\r\n");
    }
    return 0;
}
=======
>>>>>>> 1728c27369ef9e67d2a3ff2c6587be7f7b0eb2a0
