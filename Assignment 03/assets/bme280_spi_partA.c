/*
 * bme280_spi_partA.c
 * CSE 2206 — Microcontroller & Embedded System — Assignment 3, Part A
 * BME280 via SPI2 (4-wire) — Bare-Metal Register-Level — STM32F446RE Nucleo-64
 *
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │  CLOCK TREE                                                             │
 * │  HSE (8 MHz) → PLL (PLLM=4, PLLN=180, PLLP=2) → SYSCLK = 180 MHz     │
 * │  AHB /1 = 180 MHz  │  APB1 /4 = 45 MHz  │  APB2 /2 = 90 MHz           │
 * │  TIM6 clock = 2 × APB1 = 90 MHz  (APB1 prescaler ≠ 1)                 │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │  PIN MAPPING                                                            │
 * │  PC1  SPI2_MOSI   AF7        BME280 SDI                                 │
 * │  PC2  SPI2_MISO   AF5        BME280 SDO                                 │
 * │  PC7  SPI2_SCK    AF5        BME280 SCK                                 │
 * │  PB9  CS          GPIO OUT   BME280 CSB  (active-LOW)                   │
 * │  PA2  USART2_TX   AF7        USB-UART TX                                │
 * │  PA3  USART2_RX   AF7        USB-UART RX                                │
 * └─────────────────────────────────────────────────────────────────────────┘
 */

#include "stm32f446xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ── CS pin helpers ───────────────────────────────────────────────────────── */
#define CS_LOW()   (GPIOB->ODR &= ~(1UL << 9))
#define CS_HIGH()  (GPIOB->ODR |=  (1UL << 9))

/* ── BME280 register addresses ────────────────────────────────────────────── */
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_DATA      0xF7
#define BME280_CHIP_ID       0x60

/* ── SPI read/write encoding (MSB = direction bit) ───────────────────────── */
#define SPI_READ(reg)   ((reg) | 0x80U)
#define SPI_WRITE(reg)  ((reg) & 0x7FU)

/* ── Calibration coefficients ─────────────────────────────────────────────── */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t  dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;
static int32_t  t_fine;

/* ── Converted output values (written in ISR, read-only in main) ─────────── */
static volatile float g_temp_C, g_temp_F, g_pres_hPa, g_hum_RH;

/* ── ISR flags ────────────────────────────────────────────────────────────── */
volatile uint8_t sensor_ready = 0;   /* set by TIM6 ISR each second         */

/* ── Heartbeat counter for Test A3 ───────────────────────────────────────── */
static volatile uint32_t tim6_ticks = 0;

/* =========================================================================
   1.  PLL — 180 MHz SYSCLK from HSE (8 MHz on Nucleo-64 ST-Link MCO)
   ========================================================================= */
void PLL_Config(void)
{
    /* Enable HSE and wait for it to stabilise */
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));

    /*
     * PLL: HSE(8 MHz) / PLLM(4) * PLLN(180) / PLLP(2) = 180 MHz
     *       PLLQ = 4 → USB/SDIO clock = 45 MHz (not used here)
     */
    RCC->PLLCFGR = (4U   << RCC_PLLCFGR_PLLM_Pos)
                 | (180U << RCC_PLLCFGR_PLLN_Pos)
                 | (0U   << RCC_PLLCFGR_PLLP_Pos)   /* PLLP = 2 (00b) */
                 | (4U   << RCC_PLLCFGR_PLLQ_Pos)
                 | RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /*
     * Flash: 5 wait-states for 180 MHz @ VCC=3.3V (Vcore Range 1),
     *        plus instruction/data cache and prefetch enable.
     */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN
               | FLASH_ACR_LATENCY_5WS;

    /* Bus prescalers: AHB/1=180MHz  APB1/4=45MHz  APB2/2=90MHz */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1
              | RCC_CFGR_PPRE1_DIV4
              | RCC_CFGR_PPRE2_DIV2;

    /* Switch SYSCLK source to PLL */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

/* =========================================================================
   2.  Software delay — calibrated for 180 MHz, used only during init
       (TIM6 is reserved for the 1-second ISR)
   ========================================================================= */
static void delay_ms(uint32_t ms)
{
    /*
     * At 180 MHz, ~180 000 cycles/ms.
     * __NOP() ≈ 1 cycle; loop overhead ≈ 3 cycles → divisor ≈ 45 000.
     * A 2× safety margin is fine for the BME280 startup delay.
     */
    for (uint32_t i = 0; i < ms * 45000UL; i++) __NOP();
}

/* =========================================================================
   3.  TIM6 — 1-second update interrupt
       TIM6 clock = 2 × APB1 = 90 MHz  (because APB1 prescaler = /4 ≠ 1)
       PSC = 8999  →  tick frequency = 90 MHz / 9000 = 10 kHz (0.1 ms/tick)
       ARR = 9999  →  period = 10 000 × 0.1 ms = 1 second
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
   4.  GPIO Configuration
       Steps A3.2 — enable clocks, set modes, AFs, speeds
   ========================================================================= */
void GPIO_Config(void)
{
    /* Enable clocks for GPIOA (USART2), GPIOB (CS), GPIOC (SPI2) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN
                 |  RCC_AHB1ENR_GPIOBEN
                 |  RCC_AHB1ENR_GPIOCEN;
    __NOP(); __NOP();

    /* ── PA2 (USART2 TX) and PA3 (USART2 RX) → AF7 ───────────────────── */
    GPIOA->MODER  &= ~((3UL << (2*2)) | (3UL << (3*2)));
    GPIOA->MODER  |=  ((2UL << (2*2)) | (2UL << (3*2)));   /* Alternate   */
    GPIOA->OTYPER &= ~((1UL << 2) | (1UL << 3));            /* push-pull   */
    GPIOA->OSPEEDR|=  ((3UL << (2*2)) | (3UL << (3*2)));   /* high speed  */
    GPIOA->PUPDR  &= ~((3UL << (2*2)) | (3UL << (3*2)));   /* no pull     */
    GPIOA->AFR[0] &= ~((0xFUL << (4*2)) | (0xFUL << (4*3)));
    GPIOA->AFR[0] |=  ((7UL   << (4*2)) | (7UL   << (4*3)));  /* AF7      */

    /* ── PB9 (CS) → General-purpose output, push-pull, initially HIGH ─── */
    GPIOB->MODER  &= ~(3UL << (9*2));
    GPIOB->MODER  |=  (1UL << (9*2));   /* Output mode                     */
    GPIOB->OTYPER &= ~(1UL << 9);       /* push-pull                       */
    GPIOB->OSPEEDR|=  (3UL << (9*2));   /* high speed                      */
    GPIOB->PUPDR  &= ~(3UL << (9*2));   /* no pull                         */
    GPIOB->ODR    |=  (1UL << 9);       /* CS = HIGH (idle)                */

    /* ── PC1 (SPI2_MOSI) → AF7 ────────────────────────────────────────── */
    /* ── PC2 (SPI2_MISO) → AF5 ────────────────────────────────────────── */
    /* ── PC7 (SPI2_SCK)  → AF5 ────────────────────────────────────────── */
    GPIOC->MODER  &= ~((3UL << (1*2)) | (3UL << (2*2)) | (3UL << (7*2)));
    GPIOC->MODER  |=  ((2UL << (1*2)) | (2UL << (2*2)) | (2UL << (7*2)));
    GPIOC->OTYPER &= ~((1UL << 1) | (1UL << 2) | (1UL << 7));  /* push-pull */
    GPIOC->OSPEEDR|=  ((3UL << (1*2)) | (3UL << (2*2)) | (3UL << (7*2)));
    GPIOC->PUPDR  &= ~((3UL << (1*2)) | (3UL << (2*2)) | (3UL << (7*2)));

    /* AFRL covers pins 0–7 (AFR[0]):
       PC1 → bits [7:4]   = AF7 (SPI2_MOSI)
       PC2 → bits [11:8]  = AF5 (SPI2_MISO)
       PC7 → bits [31:28] = AF5 (SPI2_SCK)  */
    GPIOC->AFR[0] &= ~((0xFUL << (4*1)) | (0xFUL << (4*2)) | (0xFUL << (4*7)));
    GPIOC->AFR[0] |=  ((7UL   << (4*1)) | (5UL   << (4*2)) | (5UL   << (4*7)));
}

/* =========================================================================
   5.  USART2 — 115 200 baud, APB1 = 45 MHz
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
   6.  SPI2 — Master, Mode 00 (CPOL=0, CPHA=0), 8-bit, BR=fPCLK/8
       fSPI = APB1 / 8 = 45 / 8 ≈ 5.6 MHz  (well within BME280's 10 MHz max)
   ========================================================================= */
void SPI2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    __NOP(); __NOP();

    /*
     * CR1 layout:
     *   CPHA    [0]  = 0   data sampled on first (rising) edge
     *   CPOL    [1]  = 0   SCK idle LOW
     *   MSTR    [2]  = 1   master mode
     *   BR      [5:3]= 010 fPCLK/8
     *   SPE     [6]       (set last)
     *   LSBFIRST[7]  = 0   MSB first
     *   SSI     [8]  = 1   \
     *   SSM     [9]  = 1    software NSS — CS managed by PB9 GPIO
     *   DFF     [11] = 0   8-bit frame
     */
    SPI2->CR1 = (1U << 2)    /* MSTR    */
              | (2U << 3)    /* BR[2:0] = 010 → /8  */
              | (0U << 1)    /* CPOL = 0 */
              | (0U << 0)    /* CPHA = 0 */
              | (0U << 7)    /* LSBFIRST = 0 (MSB first) */
              | (1U << 9)    /* SSM = 1  */
              | (1U << 8)    /* SSI = 1  */
              | (0U << 11);  /* DFF = 0 (8-bit) */

    SPI2->CR1 |= (1U << 6);  /* SPE = 1 — enable SPI2 */
}

/* ── Low-level SPI byte exchange ─────────────────────────────────────────── */
static uint8_t SPI_TxRx(uint8_t data)
{
    while (!(SPI2->SR & SPI_SR_TXE));    /* wait: TX buffer empty           */
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_RXNE));   /* wait: RX buffer not empty       */
    return (uint8_t)SPI2->DR;
}

/* =========================================================================
   7.  BME280 SPI Read / Write helpers
   ========================================================================= */
static void BME280_WriteReg(uint8_t reg, uint8_t data)
{
    CS_LOW();
    SPI_TxRx(SPI_WRITE(reg));   /* MSB = 0 → write                         */
    SPI_TxRx(data);
    CS_HIGH();
}

static void BME280_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    CS_LOW();
    SPI_TxRx(SPI_READ(reg));    /* MSB = 1 → read; addr auto-increments    */
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI_TxRx(0xFF);  /* clock out data with dummy bytes       */
    CS_HIGH();
}

/* =========================================================================
   8.  BME280 Calibration Read
   ========================================================================= */
static void BME280_ReadCalibration(void)
{
    uint8_t c[24], h[7];

    /* Temperature + Pressure: registers 0x88–0x9F (24 bytes) */
    BME280_ReadRegs(0x88, c, 24);

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

    /* dig_H1: register 0xA1 */
    BME280_ReadRegs(0xA1, &dig_H1, 1);

    /* Humidity: registers 0xE1–0xE7 (7 bytes) */
    BME280_ReadRegs(0xE1, h, 7);
    dig_H2 = (int16_t)((h[1] << 8) | h[0]);
    dig_H3 = h[2];
    dig_H4 = (int16_t)((h[3] << 4) | (h[4] & 0x0F));
    dig_H5 = (int16_t)((h[5] << 4) | (h[4] >> 4));
    dig_H6 = (int8_t)h[6];
}

/* =========================================================================
   9.  BME280 Compensation Formulas (Bosch datasheet, 64-bit integer path)
   ========================================================================= */

/* Returns temperature in °C × 100 and populates t_fine */
static int32_t BME280_CompTemp(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * (int32_t)dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
              ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * (int32_t)dig_T3) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

/* Returns pressure in Pa × 256 */
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

/* Returns humidity in %RH × 1024 */
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
   10. BME280 Initialise
   ========================================================================= */
static void BME280_Init(void)
{
    uint8_t chip_id = 0;

    /* ── A3.6 Step 1: soft reset ─────────────────────────────────────────── */
    BME280_WriteReg(BME280_REG_RESET, 0xB6);
    delay_ms(10);     /* datasheet startup ≤ 2 ms; 10 ms adds safe margin    */

    /* ── A3.6 Step 2: read and verify chip ID (Test A1) ─────────────────── */
    BME280_ReadRegs(BME280_REG_ID, &chip_id, 1);

    {
        char s[64];
        sprintf(s, "[A1] ChipID=0x%02X (expect 0x60)\r\n", chip_id);
        USART2_SendString(s);
    }

    if (chip_id != BME280_CHIP_ID)
    {
        USART2_SendString("[ERR] BME280 not found — check CS polarity, CPOL/CPHA, wiring\r\n");
        while (1);
    }

    /* ── A3.6 Step 3: read calibration coefficients ─────────────────────── */
    BME280_ReadCalibration();

    /* ── A3.6 Step 4: configure operating mode (Indoor Navigation) ────────
     *  0xF2 ctrl_hum  = 0x01  osrs_h  = 001 (×1)
     *  0xF4 ctrl_meas = 0x57  osrs_t  = 010 (×2),
     *                         osrs_p  = 101 (×16),
     *                         mode    = 11  (Normal)
     *  0xF5 config    = 0x10  t_sb    = 000 (0.5 ms standby),
     *                         filter  = 100 (IIR ×16)
     *  NOTE: ctrl_hum MUST be written before ctrl_meas (BME280 datasheet §5.4.3)
     */
    BME280_WriteReg(BME280_REG_CTRL_HUM,  0x01);
    BME280_WriteReg(BME280_REG_CTRL_MEAS, 0x57);
    BME280_WriteReg(BME280_REG_CONFIG,    0x10);
}

/* =========================================================================
   11. BME280 Read, Compensate, and Store
       Called from TIM6 ISR — SPI polling is safe at low ISR rates.
   ========================================================================= */
static void BME280_ReadAll(void)
{
    uint8_t  raw[8];
    int32_t  adc_P, adc_T, adc_H;

    /* Burst-read 8 bytes from 0xF7:
       [0]=press_msb [1]=press_lsb [2]=press_xlsb
       [3]=temp_msb  [4]=temp_lsb  [5]=temp_xlsb
       [6]=hum_msb   [7]=hum_lsb                                            */
    BME280_ReadRegs(BME280_REG_DATA, raw, 8);

    /* Reconstruct 20-bit ADC values */
    adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    adc_H = ((int32_t)raw[6] << 8)  |  (int32_t)raw[7];

    /* CompTemp MUST run first — it populates t_fine used by Pres and Hum   */
    int32_t  temp_raw = BME280_CompTemp(adc_T);  /* °C × 100               */
    uint32_t pres_raw = BME280_CompPres(adc_P);  /* Pa × 256               */
    uint32_t hum_raw  = BME280_CompHum(adc_H);   /* %RH × 1024             */

    /* Convert to display units */
    g_temp_C   = (float)temp_raw / 100.0f;
    g_temp_F   = g_temp_C * 9.0f / 5.0f + 32.0f;
    g_pres_hPa = ((float)pres_raw / 256.0f) / 100.0f;
    g_hum_RH   = (float)hum_raw / 1024.0f;

    sensor_ready = 1;
}

/* =========================================================================
   12. TIM6 ISR — fires every 1 second
   ========================================================================= */
void TIM6_DAC_IRQHandler(void)
{
    if (TIM6->SR & TIM_SR_UIF)
    {
        TIM6->SR &= ~TIM_SR_UIF;   /* clear interrupt flag */
        tim6_ticks++;               /* Test A3 heartbeat counter             */
        BME280_ReadAll();           /* read + compensate sensor              */
    }
}

/* =========================================================================
   13. Verification Tests A1–A4
       (A1 is integrated into BME280_Init)
       (A5 is filled in manually by comparing bare-metal vs HAL readings)
   ========================================================================= */

/* Test A2 — UART loopback: call BEFORE BME280_Init() */
static void Test_A2_UART(void)
{
    USART2_SendString("[A2] UART OK\r\n");
}

/* Test A3 — Heartbeat: print running tick count once per second */
static void Test_A3_Heartbeat(void)
{
    char s[48];
    sprintf(s, "[A3] Tick:%lu\r\n", (unsigned long)tim6_ticks);
    USART2_SendString(s);
}

/* Test A4 — Plausibility check on live readings */
static void Test_A4_Plausibility(void)
{
    if      (g_temp_C  < 15.0f || g_temp_C  > 40.0f) USART2_SendString("[A4] Temp FAIL\r\n");
    else if (g_pres_hPa < 900.0f || g_pres_hPa > 1100.0f) USART2_SendString("[A4] Pres FAIL\r\n");
    else if (g_hum_RH  < 0.0f   || g_hum_RH  > 100.0f)   USART2_SendString("[A4] Hum  FAIL\r\n");
    else                                                    USART2_SendString("[A4] Plausibility PASS\r\n");
}

/* =========================================================================
   14. Main
   ========================================================================= */
int main(void)
{
    PLL_Config();      /* 180 MHz SYSCLK, 45 MHz APB1                       */
    GPIO_Config();     /* PA2/3 USART2, PB9 CS, PC1/2/7 SPI2                */
    USART2_Init();     /* 115 200 baud                                       */

    /* ── Test A2: UART must work before touching SPI ──────────────────── */
    Test_A2_UART();

    SPI2_Init();       /* 5.6 MHz SPI Master, Mode 00                       */
    BME280_Init();     /* reset → [A1] chip-ID → calibration → config       */

    /* ── Banner ────────────────────────────────────────────────────────── */
    USART2_SendString("========================================\r\n");
    USART2_SendString(" BME280 via SPI -- CSE 2206 Lab A\r\n");
    USART2_SendString("========================================\r\n");

    TIM6_Init();       /* start 1-second ISR                                 */

    while (1)
    {
        if (sensor_ready)
        {
            sensor_ready = 0;

            char msg[128];
            sprintf(msg,
                    "[SPI] Temp:%.2fC/%.2fF Pres:%.2fhPa Hum:%.2f%%\r\n",
                    g_temp_C, g_temp_F, g_pres_hPa, g_hum_RH);
            USART2_SendString(msg);

            Test_A3_Heartbeat();
            Test_A4_Plausibility();
            USART2_SendString("----------------------------------------\r\n");
        }

        __WFI();  /* sleep until next interrupt */
    }
}

/*
 * ── Test A5 — HAL vs Bare-Metal comparison ──────────────────────────────
 *
 * Record three consecutive readings from this bare-metal build, then
 * flash the HAL / CubeIDE version and record three more.  Fill the table:
 *
 *  Reading  │  BM Temp (°C)  │  HAL Temp (°C)  │  Diff  │  Pass (≤0.1°C)?
 *  ─────────┼────────────────┼─────────────────┼────────┼─────────────────
 *     1     │                │                 │        │
 *     2     │                │                 │        │
 *     3     │                │                 │        │
 *
 * Both builds must output values within ±0.1 °C of each other.
 * Any larger discrepancy indicates a BRR or oversampling misconfiguration.
 */
