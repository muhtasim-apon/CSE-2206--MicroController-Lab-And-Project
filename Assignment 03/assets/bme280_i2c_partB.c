/*
 * bme280_i2c_partB.c
 * CSE 2206 — Microcontroller & Embedded System — Assignment 3, Part B
 * BME280 via I2C1 (2-wire) — Bare-Metal Register-Level — STM32F446RE Nucleo-64
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  CLOCK TREE  (identical to Part A — Steps A3.1–A3.4 repeated)          │
 * │  HSE (8 MHz) → PLL (PLLM=4, PLLN=180, PLLP=2) → SYSCLK = 180 MHz     │
 * │  AHB /1 = 180 MHz  │  APB1 /4 = 45 MHz  │  APB2 /2 = 90 MHz           │
 * │  TIM6 clock = 2 × APB1 = 90 MHz  (APB1 prescaler ≠ 1)                 │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │  PIN MAPPING                                                            │
 * │  PB6  I2C1_SCL    AF4 (open-drain + pull-up)   BME280 SCK              │
 * │  PB7  I2C1_SDA    AF4 (open-drain + pull-up)   BME280 SDI/SDO          │
 * │  PA2  USART2_TX   AF7                           USB-UART TX             │
 * │  PA3  USART2_RX   AF7                           USB-UART RX             │
 * │                                                                          │
 * │  External 4.7 kΩ pull-up resistors on SDA and SCL to 3.3 V required.  │
 * │  (Many BME280 breakout boards include them on-board.)                   │
 * └─────────────────────────────────────────────────────────────────────────┘
 */

#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ── BME280 I2C address (SDO/ADDR pin → GND) ─────────────────────────────── */
#define BME280_I2C_ADDR     0x76U

/* ── BME280 register map ─────────────────────────────────────────────────── */
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_DATA      0xF7
#define BME280_CHIP_ID       0x60

/* ── Calibration coefficients (factory-programmed into BME280 NVM) ────────── */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;
static int32_t  t_fine;   /* shared between temperature, pressure, humidity  */

/* ── Converted output values (updated in ISR) ────────────────────────────── */
static volatile float g_temp_C, g_temp_F, g_pres_hPa, g_hum_RH;

/* ── Flags ───────────────────────────────────────────────────────────────── */
volatile uint8_t sensor_ready = 0;
static volatile uint32_t tim6_ticks = 0;

/* =========================================================================
   1.  PLL — 180 MHz SYSCLK from HSE 8 MHz  (Step A3.1, repeated for Part B)
   ========================================================================= */
void PLL_Config(void)
{
    /* Enable HSE */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    /*
     * PLL:  HSE(8 MHz) / PLLM(4) × PLLN(180) / PLLP(2) = 180 MHz
     */
    RCC->PLLCFGR = (4U   << RCC_PLLCFGR_PLLM_Pos)
                 | (180U << RCC_PLLCFGR_PLLN_Pos)
                 | (0U   << RCC_PLLCFGR_PLLP_Pos)   /* PLLP = 2 (00b)     */
                 | (4U   << RCC_PLLCFGR_PLLQ_Pos)
                 | RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* Flash: 5 WS for 180 MHz @ Vcore Range 1, plus caches and prefetch   */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN
               | FLASH_ACR_LATENCY_5WS;

    /* AHB /1 = 180 MHz  │  APB1 /4 = 45 MHz  │  APB2 /2 = 90 MHz         */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV4
              | RCC_CFGR_PPRE2_DIV2;

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* =========================================================================
   2.  Software delay — calibrated for 180 MHz core, init use only
   ========================================================================= */
static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 45000UL; i++) __NOP();
}

/* =========================================================================
   3.  TIM6 — 1-second update interrupt  (Step A3.4, repeated for Part B)
       TIM6 clock = 2 × APB1 = 90 MHz
       PSC = 8999  →  10 kHz tick (0.1 ms per count)
       ARR = 9999  →  10 000 × 0.1 ms = 1 second
   ========================================================================= */
void TIM6_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    __NOP(); __NOP();

    TIM6->CR1  &= ~TIM_CR1_CEN;
    TIM6->PSC   = 8999U;
    TIM6->ARR   = 9999U;
    TIM6->EGR  |= TIM_EGR_UG;
    TIM6->SR    = 0;
    TIM6->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1  |= TIM_CR1_CEN;
}

/* =========================================================================
   4.  GPIO Configuration  (Step A3.2 + Step B3.1)
   ========================================================================= */
void GPIO_Config(void)
{
    /* ── Enable GPIOA (USART2) and GPIOB (I2C1) clocks ──────────────────
     * NOTE: Use |= to avoid wiping other bits already set in AHB1ENR.
     * The original submitted code used = (assignment) which is a bug.      */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    __NOP(); __NOP();

    /* ── Step A3.2: PA2 (USART2_TX) and PA3 (USART2_RX) → AF7 ──────── */
    GPIOA->MODER  &= ~((3UL << (2*2)) | (3UL << (3*2)));
    GPIOA->MODER  |=  ((2UL << (2*2)) | (2UL << (3*2)));   /* Alternate  */
    GPIOA->OTYPER &= ~((1UL << 2) | (1UL << 3));
    GPIOA->OSPEEDR|=  ((3UL << (2*2)) | (3UL << (3*2)));   /* high speed */
    GPIOA->PUPDR  &= ~((3UL << (2*2)) | (3UL << (3*2)));
    GPIOA->AFR[0] &= ~((0xFUL << (4*2)) | (0xFUL << (4*3)));
    GPIOA->AFR[0] |=  ((7UL   << (4*2)) | (7UL   << (4*3)));

    /* ── Step B3.1: PB6 (I2C1_SCL) and PB7 (I2C1_SDA) → AF4 ──────────
     *
     *  MODER   = 10  (Alternate Function)
     *  OTYPER  = 1   (open-drain) — MANDATORY for I2C bus compliance
     *  OSPEEDR = 01  (medium speed) — adequate for 100 kHz
     *  PUPDR   = 01  (pull-up) — supplement external 4.7 kΩ resistors
     *  AF4 set in AFRL (AFR[0]) for both pins                              */
    GPIOB->MODER  &= ~((3UL << (6*2)) | (3UL << (7*2)));
    GPIOB->MODER  |=  ((2UL << (6*2)) | (2UL << (7*2)));

    GPIOB->AFR[0] &= ~((0xFUL << (4*6)) | (0xFUL << (4*7)));
    GPIOB->AFR[0] |=  ((4UL   << (4*6)) | (4UL   << (4*7)));

    GPIOB->OTYPER |=  (1UL << 6) | (1UL << 7);             /* open-drain  */

    GPIOB->OSPEEDR &= ~((3UL << (6*2)) | (3UL << (7*2)));
    GPIOB->OSPEEDR |=  ((1UL << (6*2)) | (1UL << (7*2)));  /* medium     */

    GPIOB->PUPDR  &= ~((3UL << (6*2)) | (3UL << (7*2)));
    GPIOB->PUPDR  |=  ((1UL << (6*2)) | (1UL << (7*2)));   /* pull-up    */
}

/* =========================================================================
   5.  USART2 — 115 200 baud, APB1 = 45 MHz  (Step A3.3, repeated)
       BRR: USARTDIV = 45 000 000 / (16 × 115 200) ≈ 24.41
            Mantissa = 24, Fraction = round(0.41 × 16) = 7
   ========================================================================= */
void USART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    __NOP(); __NOP();

    USART2->BRR  = (24UL << 4) | 7UL;
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void USART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

void USART2_SendString(const char *str)
{
    while (*str) USART2_SendChar(*str++);
}

/* =========================================================================
   6.  I2C1 Peripheral — Standard Mode 100 kHz, APB1 = 45 MHz  (Step B3.2)
   ========================================================================= */
void I2C1_Init(void)
{
    /* Enable I2C1 clock: RCC APB1ENR bit 21 */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    __NOP(); __NOP();

    /* Reset then release: RCC APB1RSTR bit 21
     * BUG in submitted code: used non-existent register RCC->APB1RST       */
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    I2C1->CR1 = 0;      /* disable I2C (PE=0) before configuring            */

    /*
     * CR2 FREQ field = APB1 frequency in MHz = 45
     *
     * CCR — Standard Mode (Sm), 100 kHz:
     *   T_high = T_low = 5 µs
     *   CCR = T_high / T_pclk1 = 5000 ns / (1/45 MHz) = 225
     *
     * TRISE — max rise time in Sm = 1000 ns:
     *   TRISE = floor(1000 ns × 45 MHz) + 1 = 45 + 1 = 46
     *
     * BUG in submitted code: wrote to I2C1_CCR (bare name, no ->)
     */
    I2C1->CR2   = 45U;
    I2C1->CCR   = 225U;
    I2C1->TRISE = 46U;

    I2C1->CR1 |= I2C_CR1_PE;   /* PE = 1 — enable I2C1 */
}

/* =========================================================================
   7.  I2C1 Write — single byte to one register  (B4.1)
   ========================================================================= */
void I2C1_WriteReg(uint8_t reg, uint8_t data)
{
    /* Generate START */
    I2C1->CR1 |= I2C_CR1_START;
    /* BUG in submitted code: polled I2C1->CR1 bit 0 (PE) instead of
     * I2C1->SR1 SB flag. CR1[0] is the PE enable bit, not start-bit status.*/
    while (!(I2C1->SR1 & I2C_SR1_SB));

    /* Send 7-bit slave address + WRITE (R/W bit = 0) */
    I2C1->DR = (BME280_I2C_ADDR << 1) | 0U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    /* Clear ADDR flag by reading SR1 then SR2.
     * BUG in submitted code: wrote 0 to SR2; a write has no defined effect.
     * The flag is cleared only by a read of SR2 after SR1.                  */
    (void)I2C1->SR2;

    /* Send register address */
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    /* Send data byte */
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));   /* BTF = both byte transfer done */

    /* Generate STOP */
    I2C1->CR1 |= I2C_CR1_STOP;
}

/* =========================================================================
   8.  I2C1 Burst Read — set register pointer, repeated START, read N bytes
       (B4.2)
   ========================================================================= */
void I2C1_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* ── Phase 1: write register pointer ────────────────────────────────── */
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (BME280_I2C_ADDR << 1) | 0U;   /* addr + WRITE              */
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    I2C1->DR = reg;
    /* BUG in submitted code: jumped straight to BTF without waiting TXE.
     * TXE must be set first (byte loaded into shift register) before BTF
     * confirms the full transfer is complete. Skipping TXE risks a race.   */
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    /* ── Phase 2: repeated START, switch to read mode ───────────────────── */
    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_START;  /* ACK=1, issue RESTART     */
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (BME280_I2C_ADDR << 1) | 1U;    /* addr + READ              */
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    for (uint8_t i = 0; i < len; i++)
    {
        if (i == len - 1)
            I2C1->CR1 &= ~I2C_CR1_ACK;   /* send NACK before last byte     */

        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        buf[i] = (uint8_t)I2C1->DR;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
}

/* =========================================================================
   9.  BME280 Calibration Read
   ========================================================================= */
static void BME280_ReadCalibration(void)
{
    uint8_t c[24], h[7];

    /* Temperature + Pressure calibration: 0x88–0x9F = 24 bytes */
    I2C1_ReadRegs(0x88, c, 24);

    dig_T1 = (uint16_t)((c[1]  << 8) | c[0]);
    dig_T2 = (int16_t) ((c[3]  << 8) | c[2]);
    dig_T3 = (int16_t) ((c[5]  << 8) | c[4]);
    dig_P1 = (uint16_t)((c[7]  << 8) | c[6]);
    dig_P2 = (int16_t) ((c[9]  << 8) | c[8]);
    dig_P3 = (int16_t) ((c[11] << 8) | c[10]);
    dig_P4 = (int16_t) ((c[13] << 8) | c[12]);
    dig_P5 = (int16_t) ((c[15] << 8) | c[14]);
    dig_P6 = (int16_t) ((c[17] << 8) | c[16]);
    dig_P7 = (int16_t) ((c[19] << 8) | c[18]);
    dig_P8 = (int16_t) ((c[21] << 8) | c[20]);
    dig_P9 = (int16_t) ((c[23] << 8) | c[22]);

    /* dig_H1: register 0xA1 (1 byte) */
    I2C1_ReadRegs(0xA1, &dig_H1, 1);

    /* Humidity calibration: 0xE1–0xE7 (7 bytes) */
    I2C1_ReadRegs(0xE1, h, 7);
    dig_H2 = (int16_t)((h[1] << 8) | h[0]);
    dig_H3 = h[2];
    dig_H4 = (int16_t)((h[3] << 4) | (h[4] & 0x0F));
    dig_H5 = (int16_t)((h[5] << 4) | (h[4] >> 4));
    dig_H6 = (int8_t)h[6];
}

/* =========================================================================
   10. BME280 Compensation Formulas (Bosch datasheet, integer path)
   ========================================================================= */

/* Temperature — returns °C × 100; populates t_fine */
static int32_t BME280_CompTemp(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
              ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

/* Pressure — returns Pa × 256 */
static uint32_t BME280_CompPres(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = (int64_t)t_fine - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + ((int64_t)dig_P4 << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = ((((int64_t)1 << 47) + var1) * (int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0;
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)dig_P8 * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7 << 4);
    return (uint32_t)p;
}

/* Humidity — returns %RH × 1024 */
static uint32_t BME280_CompHum(int32_t adc_H)
{
    int32_t v;
    v = t_fine - 76800;
    v = (((((adc_H << 14) - ((int32_t)dig_H4 << 20) - ((int32_t)dig_H5 * v))
            + 16384) >> 15) *
         (((((((v * (int32_t)dig_H6) >> 10) *
              (((v * (int32_t)dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
           (int32_t)dig_H2 + 8192) >> 14));
    v -= (((((v >> 15) * (v >> 15)) >> 7) * (int32_t)dig_H1) >> 4);
    if (v < 0)         v = 0;
    if (v > 419430400) v = 419430400;
    return (uint32_t)(v >> 12);
}

/* =========================================================================
   11. BME280 Initialise  (Step B3.3)
   ========================================================================= */
static void BME280_Init(void)
{
    uint8_t chip_id = 0;

    /* Soft reset: write 0xB6 to register 0xE0 */
    I2C1_WriteReg(BME280_REG_RESET, 0xB6);
    delay_ms(10);

    /* Read chip ID — Test B1 integrated here */
    I2C1_ReadRegs(BME280_REG_ID, &chip_id, 1);
    {
        char s[64];
        sprintf(s, "[B1] ChipID=0x%02X (expect 0x60)\r\n", chip_id);
        USART2_SendString(s);
    }
    if (chip_id != BME280_CHIP_ID)
    {
        USART2_SendString("[ERR] BME280 not found — check SDA/SCL pull-ups, address (0x76 vs 0x77), OTYPER open-drain\r\n");
        while (1);
    }

    /* Load calibration coefficients */
    BME280_ReadCalibration();

    /* Configure operating mode (Indoor Navigation, from Section 4.3):
     *
     *  0xF2 ctrl_hum  = 0x01   osrs_h  = 001  (×1)
     *  0xF4 ctrl_meas = 0x57   osrs_t  = 010  (×2)
     *                          osrs_p  = 101  (×16)
     *                          mode    = 11   (Normal)
     *  0xF5 config    = 0x10   t_sb    = 000  (0.5 ms standby)
     *                          filter  = 100  (IIR ×16)
     *
     *  IMPORTANT: ctrl_hum must be written BEFORE ctrl_meas (§5.4.3).     */
    I2C1_WriteReg(BME280_REG_CTRL_HUM,  0x01);
    I2C1_WriteReg(BME280_REG_CTRL_MEAS, 0x57);
    I2C1_WriteReg(BME280_REG_CONFIG,    0x10);
}

/* =========================================================================
   12. BME280 Read, Compensate, Store  (Step B3.4)
   ========================================================================= */
static void BME280_ReadAll(void)
{
    uint8_t raw[8];
    int32_t adc_P, adc_T, adc_H;

    /* Burst-read 8 bytes from 0xF7 (Step B3.4) */
    I2C1_ReadRegs(BME280_REG_DATA, raw, 8);

    /* ADC reconstruction (same as Part A, Section A3.7) */
    adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    adc_H = ((int32_t)raw[6] << 8)  |  (int32_t)raw[7];

    /* CompTemp MUST run first — it populates t_fine */
    int32_t  temp_raw = BME280_CompTemp(adc_T);   /* °C × 100             */
    uint32_t pres_raw = BME280_CompPres(adc_P);   /* Pa × 256             */
    uint32_t hum_raw  = BME280_CompHum(adc_H);    /* %RH × 1024           */

    g_temp_C   = (float)temp_raw / 100.0f;
    g_temp_F   = g_temp_C * 9.0f / 5.0f + 32.0f;
    g_pres_hPa = ((float)pres_raw / 256.0f) / 100.0f;
    g_hum_RH   = (float)hum_raw / 1024.0f;

    sensor_ready = 1;
}

/* =========================================================================
   13. TIM6 ISR — fires every 1 second (Step B3.4)
   ========================================================================= */
void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF)
    {
        TIM6->SR &= ~TIM_SR_UIF;
        tim6_ticks++;
        BME280_ReadAll();
    }
}

/* =========================================================================
   14. Verification Tests B1–B4
       B1 is integrated into BME280_Init() above.
   ========================================================================= */

/* Test B2 — I2C ACK probe: confirm the sensor responds on the bus */
static void Test_B2_I2C_ACK(void)
{
    uint32_t timeout = 100000U;

    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->DR = (BME280_I2C_ADDR << 1) | 0U;

    while (!(I2C1->SR1 & I2C_SR1_ADDR) && --timeout);

    if (!timeout)
    {
        USART2_SendString("[B2] I2C ACK FAIL — check pull-ups and address\r\n");
    }
    else
    {
        (void)I2C1->SR2;
        USART2_SendString("[B2] I2C ACK OK\r\n");
    }
    I2C1->CR1 |= I2C_CR1_STOP;
    delay_ms(1);
}

/* Test B3 — TIM6 heartbeat */
static void Test_B3_Heartbeat(void)
{
    char s[48];
    sprintf(s, "[B3] Tick:%lu\r\n", (unsigned long)tim6_ticks);
    USART2_SendString(s);
}

/* Test B4 — Sensor plausibility */
static void Test_B4_Plausibility(void)
{
    if      (g_temp_C   < 15.0f  || g_temp_C   > 40.0f)  USART2_SendString("[B4] Temp FAIL\r\n");
    else if (g_pres_hPa < 900.0f || g_pres_hPa > 1100.0f) USART2_SendString("[B4] Pres FAIL\r\n");
    else if (g_hum_RH   < 0.0f   || g_hum_RH   > 100.0f)  USART2_SendString("[B4] Hum  FAIL\r\n");
    else                                                     USART2_SendString("[B4] Plausibility PASS\r\n");
}

/* =========================================================================
   15. Main
   ========================================================================= */
int main(void)
{
    PLL_Config();    /* 180 MHz SYSCLK — identical to Part A                */
    GPIO_Config();   /* PA2/3 USART2, PB6/7 I2C1 open-drain AF4            */
    USART2_Init();   /* 115 200 baud, APB1 = 45 MHz                        */

    /* ── B2: ACK check before full init ──────────────────────────────── */
    Test_B2_I2C_ACK();

    I2C1_Init();     /* 100 kHz Sm, APB1 = 45 MHz                          */
    BME280_Init();   /* reset → [B1] chip-ID → calibration → config         */

    USART2_SendString("========================================\r\n");
    USART2_SendString(" BME280 via I2C -- CSE 2206 Lab B\r\n");
    USART2_SendString("========================================\r\n");

    TIM6_Init();     /* 1-second ISR                                        */

    while (1)
    {
        if (sensor_ready)
        {
            sensor_ready = 0;

            /* B4.3 UART output format */
            char msg[128];
            sprintf(msg,
                    "[I2C] Temp:%.2fC/%.2fF Pres:%.2fhPa Hum:%.2f%%\r\n",
                    g_temp_C, g_temp_F, g_pres_hPa, g_hum_RH);
            USART2_SendString(msg);

            Test_B3_Heartbeat();
            Test_B4_Plausibility();
            USART2_SendString("----------------------------------------\r\n");
        }

        __WFI();
    }
}

/*
 * ── Test B5 — HAL vs Bare-Metal comparison (filled manually) ─────────────
 *
 *  Reading  │  BM Temp (°C)  │  HAL Temp (°C)  │  Diff  │  Pass (≤0.1°C)?
 *  ─────────┼────────────────┼─────────────────┼────────┼─────────────────
 *     1     │                │                 │        │
 *     2     │                │                 │        │
 *     3     │                │                 │        │
 *
 * If values diverge by more than 0.1 °C, verify that both builds use
 * osrs_t = ×2 (0x57 in ctrl_meas) and the same IIR filter setting.
 */
