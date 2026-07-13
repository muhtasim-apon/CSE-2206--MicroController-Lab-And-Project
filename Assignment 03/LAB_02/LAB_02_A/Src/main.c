/* LAB 02 (Bare-Metal): BME/P-280 Sensor Communication by I2C */

#include "stm32f446xx.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* 1. Pin Configuration:
 *
 * VCC - 3V3 Pin on Nucleo
 * GND - GND Pin on Nucleo
 * SCL - SCL/D15 on Nucleo
 * SDA - SDA/D14 on Nucleo
 * CSB - Parallel Wired with VCC of Sensor
 * SDO - Parallel Wired with GND of Sensor (This enforces 0x76 as the I2C1 Address)
 *
 * */

/* 2. Register Mapping: */

#define BME_P280_REGISTER_CHIP_ID      				0xD0
#define BME_P280_REGISTER_RESET        				0xE0
#define BME_P280_REGISTER_CTRL_HUM   				0xF2
#define BME_P280_REGISTER_STATUS       				0xF3
#define BME_P280_REGISTER_CONTROL_MEASURE    		0xF4
#define BME_P280_REGISTER_CONFIGURE       			0xF5
#define BME_P280_REGISTER_PRESSURE_MSB    			0xF7
#define BME_P280_REGISTER_HUM_MSB     				0xFD
#define BME_P280_REGISTER_HUM_LSB     				0xFE
#define BME_P280_REGISTER_CALIBRATION_START  		0x88
#define BME_P280_REGISTER_HUM_CALIB   				0xA1   // extra calibration byte
#define BME_P280_REGISTER_HUM_CALIB2  				0xE1   // block of humidity calibration
#define BME_P280_RESET_VALUE      					0xB6
#define BME_P280_CTRL_HUM_VALUE      				0x01   // oversampling x1 (minimum)
#define BME_P280_CONTROL_MEASURE_VALUE    			((2U << 5) | (5U << 2) | 3U)   // Decodes to 0x57
#define BME_P280_CONFIGURE_VALUE       				((4U << 5) | (4U << 2))        // Decodes to 0x90

/* 3. Address Mapping: */

#define BME_P280_I2C_ADDRESS         				0x76
#define CHIP_ADDRESS_BME280          				0x60
#define CHIP_ADDRESS_BMP280          				0x58

/* 4. Structure for Calibration: */

typedef struct {
    uint16_t TEMP_1;
    int16_t  TEMP_2;
    int16_t  TEMP_3;
    uint16_t PRESS_1;
    int16_t  PRESS_2;
    int16_t  PRESS_3;
    int16_t  PRESS_4;
    int16_t  PRESS_5;
    int16_t  PRESS_6;
    int16_t  PRESS_7;
    int16_t  PRESS_8;
    int16_t  PRESS_9;
    uint8_t  HUM_1;
    int16_t  HUM_2;
    uint8_t  HUM_3;
    int16_t  HUM_4;
    int16_t  HUM_5;
    int8_t   HUM_6;
} BME_P280_CALIBRATION;

static BME_P280_CALIBRATION CALIBRATION;
static int32_t t_fine;

void PLL_CONFIG(void)
{
    // Configure Flash Latency for PLL_CLK = 180MHz at Access Control Register
	FLASH -> ACR =
			FLASH_ACR_ICEN | // Instruction Cache Enable
			FLASH_ACR_DCEN | // Data Cache Enable
			FLASH_ACR_PRFTEN | // Flash Pre-fetch Enable for Faster Operation
			FLASH_ACR_LATENCY_5WS; // Latency of 5 Wait States as per state cover approximately 30MHz

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

void GPIO_CONFIG(void)
{
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // clock for GPIOA Ports
	__NOP(); __NOP(); // delay for clock initiation
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // clock for GPIOB Ports
	__NOP(); __NOP(); // delay for clock initiation

	/* PA2 as USART2 TX (RX not needed in I2C Communication */
	GPIOA -> MODER &= ~(3UL << 2*2); // Reset MODE Register of GPIOA of PA2
	GPIOA -> MODER |= (2UL << 2*2); // Set MODE Register to Alternate-Function Mode for PA2(TX)

	GPIOA -> AFR[0] &= ~(0xF << 2*4); // Clearing the AFR Register Low for PA2 (each take 4 bits)
	GPIOA -> AFR[0] |= (7UL << 2*4); // Set the AFR Register Low in PA2 Position to USART2 TX/RX

	GPIOA -> OTYPER &= ~(1UL << 2); // Set output type to push-pull -> 0b0
	GPIOA -> OSPEEDR |= (3UL << 2*2); // Set Output Speed at very high -> 0b11
	GPIOA -> PUPDR &= ~(3UL << 2*2); // Set pull-up pull down to no pull-up no pull-down -> 0b00

    /* PB8 as I2C SCL, AF4, open-drain mode */
    GPIOB -> MODER &= ~(3UL << 8*2); // Reset MODE Register of GPIOB of PB8
    GPIOB -> MODER |= (2UL << 8*2); // Set MODE Register to Alternate-Function Mode for PB8 (I2C SCL)

    GPIOB -> AFR[1] &= ~(0xF << (8-8)*4); // Clearing the AFR Register High for PB8 (each take 4 bits, high ones start from 8,9,10,... in [0],[4],[8] etc.)
    GPIOB -> AFR[1] |= (4UL << (8-8)*4); // Set the AFR Register High in PB8 Position to AF4 (I2C Communication)

    GPIOB -> OTYPER &= ~(1UL << 8); // Reset output type
    GPIOB -> OTYPER |= (1U << 8); // Set Output Type to Open-Drain Mode (Line Stays High when idle)

    GPIOB -> OSPEEDR |= (1U << (8*2)); // Set Output Speed at Medium -> 0b01
    GPIOB -> PUPDR &= ~(3UL << 8*2); // Reset Pull-Up Down Register for PB8
    GPIOB -> PUPDR |= (1U << (8*2)); // Set Pull-Up Mode for PB8 (So line will stay HIGH)

    /* PB9 as I2C SDA, AF4, open-drain mode */
    GPIOB -> MODER &= ~(3UL << 9*2); // Reset MODE Register of GPIOB of PB9
	GPIOB -> MODER |= (2UL << 9*2); // Set MODE Register to Alternate-Function Mode for PB9 (I2C SDA)

	GPIOB -> AFR[1] &= ~(0xF << (9-8)*4); // Clearing the AFR Register High for PB9 (each take 4 bits, high ones start from 8,9,10,... in [0],[4],[8] etc.)
	GPIOB -> AFR[1] |= (4UL << (9-8)*4); // Set the AFR Register High in PB9 Position to AF4 (I2C Communication)

	GPIOB -> OTYPER &= ~(1UL << 9); // Reset output type
	GPIOB -> OTYPER |= (1U << 9); // Set Output Type to Open-Drain Mode (Line Stays High when idle)

	GPIOB -> OSPEEDR |= (1U << (9*2)); // Set Output Speed at Medium -> 0b01
	GPIOB -> PUPDR &= ~(3UL << 9*2); // Reset Pull-Up Down Register for PB9
	GPIOB -> PUPDR |= (1U << (9*2)); // Set Pull-Up Mode for PB9 (So line will stay HIGH)
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
    USART2 -> CR1 = USART_CR1_TE | USART_CR1_UE;
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

void I2C_CONFIG(void)
{
    RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN; // Clock for I2C Peripheral
    RCC -> APB1RSTR |= RCC_APB1RSTR_I2C1RST; // Reset the Peripheral from Peripheral Reset Register
    RCC -> APB1RSTR &= ~RCC_APB1RSTR_I2C1RST; // Clearing Reset Bit for a fresh Peripheral Start

    I2C1 -> CR2 = 45U; // As APB1 Clock = 45MHz
    I2C1 -> CCR = 225U; // For Standard mode (100 kHz), CCR = CR2*1000000/(2*100*1000) */
    I2C1 -> TRISE = 46U; // Maximum Rise Time for I2C Communication = f(APB1) + 1
    I2C1 -> CR1 |= I2C_CR1_PE; // Enabling Peripheral for Communication
}

void I2C_WRITE_BYTE(uint8_t reg, uint8_t data)
{
    I2C1 -> CR1 |= I2C_CR1_START; // Send START Condition
    while (!(I2C1 -> SR1 & I2C_SR1_SB)); // Wait until START Condition has been sent

    I2C1 -> DR = (BME_P280_I2C_ADDRESS << 1) | 0; // Send the 7-bit Slave Address and 0 as LSB for WRITE Mode
    while (!(I2C1 -> SR1 & I2C_SR1_ADDR)); // Wait until ADDR Flag Set when Slave ACKs
    (void)I2C1 -> SR2; // Clear ADDR flag by reading SR2

    I2C1 -> DR = reg; // Send the Register Address to the Slave
    while (!(I2C1 -> SR1 & I2C_SR1_TXE)); // Wait until Transmit Buffer is empty for the data

    I2C1 -> DR = data; // Send the data to the slave
    while (!(I2C1 -> SR1 & I2C_SR1_BTF)); // Wait until Byte Transmission is finished

    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP Condition
}

uint8_t I2C_READ_BYTE(uint8_t reg)
{
    uint8_t val = 0xFF; // Buffer for Read value

    I2C1 -> CR1 |= I2C_CR1_START; // Send START Condition
    while (!(I2C1 -> SR1 & I2C_SR1_SB)); // Wait until START Condition has been sent

    I2C1 -> DR = (BME_P280_I2C_ADDRESS << 1) | 0; // Send the 7-bit Slave Address and 0 as LSB for WRITE Mode
    while (!(I2C1 -> SR1 & I2C_SR1_ADDR)); // Wait until ADDR Flag Set when Slave ACKs
    (void)I2C1 -> SR2; // Clear ADDR flag by reading SR2

    I2C1 -> DR = reg; // Send the Register Address to the Slave
    while (!(I2C1 -> SR1 & I2C_SR1_BTF)); // Wait until Byte Transmission is finished

    I2C1 -> CR1 |= I2C_CR1_START; // A Repeated START Condition for Reading
    while (!(I2C1 -> SR1 & I2C_SR1_SB)); // Wait until START Condition has been sent

    I2C1 -> CR1 &= ~I2C_CR1_ACK; // Clear the ACK Flag to ensure NACK immediately (only 1 byte coming)
    I2C1 -> DR = (BME_P280_I2C_ADDRESS << 1) | 1; // Send the 7-bit Slave Address and 1 as LSB for READ Mode
    while (!(I2C1 -> SR1 & I2C_SR1_ADDR)); // Wait until ADDR Flag Set when Slave ACKs
    (void)I2C1 -> SR2; // Clear ADDR flag by reading SR2

    I2C1 -> CR1 |= I2C_CR1_STOP; // Queue STOP for last byte reading (needed for STM)
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // Wait until Receive Buffer not Empty (data found)

    val = (uint8_t)I2C1 -> DR; // Read the data
    return val;
}

void I2C_READ_BYTES(uint8_t start_reg, uint8_t *buf, uint8_t len)
{
    if (len == 0) return;

    I2C1 -> CR1 |= I2C_CR1_START; // START Cond.
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for START Cond. Sent

    I2C1 -> DR = (BME_P280_I2C_ADDRESS << 1) | 0; // 7-bit Slave Addr + 0 for WRITE
    while (!(I2C1 -> SR1 & I2C_SR1_ADDR)); // Wait until ADDR Flag Set when Slave ACKs
    (void)I2C1 -> SR2; // Clear ADDR flag by reading SR2

    I2C1 -> DR = start_reg; // Send the First Register Address to Slave
    while (!(I2C1 -> SR1 & I2C_SR1_BTF)); // Wait until Byte Tranf. finished

    I2C1 -> CR1 |= I2C_CR1_START; // A Repeated START Condition for Reading
    while (!(I2C1 -> SR1 & I2C_SR1_SB)); // Wait until START Condition has been sent

    I2C1 -> CR1 |= I2C_CR1_ACK; // Send ACK as more than 1 byte will come
    I2C1 -> DR = (BME_P280_I2C_ADDRESS << 1) | 1; // Send the 7-bit Slave Address and 1 as LSB for READ Mode
    while (!(I2C1 -> SR1 & I2C_SR1_ADDR)); // Wait until ADDR Flag Set when Slave ACKs
    (void)I2C1 -> SR2; // Clear ADDR flag by reading SR2

    for (uint8_t i = 0; i < len; i++)
    {
        if (i == (len - 1))
        {
            I2C1 -> CR1 &= ~I2C_CR1_ACK; // Send an NACK for the last byte
            I2C1 -> CR1 |=  I2C_CR1_STOP; // Generate STOP Condition for the STM
        }
        while (!(I2C1 -> SR1 & I2C_SR1_RXNE)); // Wait until Receive Buffer not Empty (data found)
        buf[i] = (uint8_t)I2C1 -> DR; // Write to the buffer pointer
    }
}

int I2C_DEVICE_ALIVE(void)
{
    I2C1 -> CR1 |= I2C_CR1_START; // START Cond.
    while (!(I2C1 -> SR1 & I2C_SR1_SB)); // Wait for START Cond. Sent

    I2C1 -> DR = (BME_P280_I2C_ADDRESS << 1) | 0; // 7-bit slave addr. + write mode

    uint32_t timeout = 200000; // Checking time
    while (!(I2C1 -> SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && --timeout); // Checking if either ACK or AF Flag sets or not to ensure device's availability

    int alive = (I2C1 -> SR1 & I2C_SR1_ADDR) != 0; // Boolean to check if Slave ACKs
    if (alive)
        (void)I2C1 -> SR2; // Clear ADDR Flag
    else
        I2C1 -> SR1 &= ~I2C_SR1_AF; // Clear AF Flag -> NACK

    I2C1 -> CR1 |= I2C_CR1_STOP; // STOP cond.
    delay_ms(2);
    return alive;
}

void BME_P280_CALIBRATION_READ(void)
{
    uint8_t CALIB_VAL[24];
    I2C_READ_BYTES(BME_P280_REGISTER_CALIBRATION_START, CALIB_VAL, 24); // Load 24-byte calibration block starting at 0x88

    CALIBRATION.TEMP_1 = (uint16_t)((CALIB_VAL[1]  << 8) | CALIB_VAL[0]);
    CALIBRATION.TEMP_2 = (int16_t) ((CALIB_VAL[3]  << 8) | CALIB_VAL[2]);
    CALIBRATION.TEMP_3 = (int16_t) ((CALIB_VAL[5]  << 8) | CALIB_VAL[4]);

    CALIBRATION.PRESS_1 = (uint16_t)((CALIB_VAL[7]  << 8) | CALIB_VAL[6]);
    CALIBRATION.PRESS_2 = (int16_t) ((CALIB_VAL[9]  << 8) | CALIB_VAL[8]);
    CALIBRATION.PRESS_3 = (int16_t) ((CALIB_VAL[11] << 8) | CALIB_VAL[10]);
    CALIBRATION.PRESS_4 = (int16_t) ((CALIB_VAL[13] << 8) | CALIB_VAL[12]);
    CALIBRATION.PRESS_5 = (int16_t) ((CALIB_VAL[15] << 8) | CALIB_VAL[14]);
    CALIBRATION.PRESS_6 = (int16_t) ((CALIB_VAL[17] << 8) | CALIB_VAL[16]);
    CALIBRATION.PRESS_7 = (int16_t) ((CALIB_VAL[19] << 8) | CALIB_VAL[18]);
    CALIBRATION.PRESS_8 = (int16_t) ((CALIB_VAL[21] << 8) | CALIB_VAL[20]);
    CALIBRATION.PRESS_9 = (int16_t) ((CALIB_VAL[23] << 8) | CALIB_VAL[22]);

    CALIBRATION.HUM_1 = I2C_READ_BYTE(BME_P280_REGISTER_HUM_CALIB);
	uint8_t humCalib[7];
	I2C_READ_BYTES(BME_P280_REGISTER_HUM_CALIB2, humCalib, 7);

	CALIBRATION.HUM_2 = (int16_t)((humCalib[1] << 8) | humCalib[0]);
	CALIBRATION.HUM_3 = humCalib[2];
	CALIBRATION.HUM_4 = (int16_t)((humCalib[3] << 4) | (humCalib[4] & 0x0F));
	CALIBRATION.HUM_5 = (int16_t)((humCalib[5] << 4) | (humCalib[4] >> 4));
	CALIBRATION.HUM_6 = (int8_t)humCalib[6];
}

void BME_P280_INIT(void)
{
    // 1. Reset the sensor (soft reset command 0xB6 to register 0xE0)
    I2C_WRITE_BYTE(BME_P280_REGISTER_RESET, BME_P280_RESET_VALUE);
    delay_ms(10); // Wait for reset to complete

    // 2. Configure general settings (IIR filter, standby time, etc.)
    // Value 0x90 = filter coefficient x16, standby time 125 ms
    I2C_WRITE_BYTE(BME_P280_REGISTER_CONFIGURE, BME_P280_CONFIGURE_VALUE);
    delay_ms(2); // Short delay for register update
    I2C_WRITE_BYTE(BME_P280_REGISTER_CTRL_HUM, BME_P280_CTRL_HUM_VALUE);
    delay_ms(2);

    // 3. Configure measurement control (oversampling + mode)
    // Value 0x57 = Temp oversampling x2, Pressure oversampling x5, Normal mode
    I2C_WRITE_BYTE(BME_P280_REGISTER_CONTROL_MEASURE, BME_P280_CONTROL_MEASURE_VALUE);
    delay_ms(2); // Short delay for register update
}

int32_t BME_P280_BOSCH_TEMP_COMPENSATE(int32_t adc_T)
{
    int32_t var1, var2, T;

    // First correction term: linear offset/slope adjustment
    var1 = ((((adc_T >> 3) - ((int32_t)CALIBRATION.TEMP_1 << 1)))
             * ((int32_t)CALIBRATION.TEMP_2)) >> 11;

    // Second correction term: quadratic correction for non-linearity
    var2 = (((((adc_T >> 4) - ((int32_t)CALIBRATION.TEMP_1))
             * ((adc_T >> 4) - ((int32_t)CALIBRATION.TEMP_1))) >> 12)
             * ((int32_t)CALIBRATION.TEMP_3)) >> 14;

    // Fine temperature value (global correction factor reused in pressure/humidity)
    t_fine = var1 + var2;

    // Final compensated temperature in 0.01 °C units
    T = (t_fine * 5 + 128) >> 8;

    return T;
}

uint32_t BME_P280_BOSCH_PRESS_COMPENSATE(int32_t adc_P)
{
    int64_t var1, var2, p;

    // Fine temperature correction
    var1 = ((int64_t)t_fine) - 128000;

    // Higher-order polynomial corrections
    var2 = var1 * var1 * (int64_t)CALIBRATION.PRESS_6;
    var2 = var2 + ((var1 * (int64_t)CALIBRATION.PRESS_5) << 17);
    var2 = var2 + (((int64_t)CALIBRATION.PRESS_4) << 35);

    // More corrections using PRESS_2 and PRESS_3
    var1 = ((var1 * var1 * (int64_t)CALIBRATION.PRESS_3) >> 8)
         + ((var1 * (int64_t)CALIBRATION.PRESS_2) << 12);

    // Scaling with PRESS_1 (main pressure calibration factor)
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)CALIBRATION.PRESS_1) >> 33;

    // Guard against division by zero
    if (var1 == 0) return 0;

    // Pressure calculation
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    // PRESS_9 and PRESS_8 corrections
    var1 = (((int64_t)CALIBRATION.PRESS_9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)CALIBRATION.PRESS_8) * p) >> 19;

    // Final adjustment with PRESS_7
    p = ((p + var1 + var2) >> 8) + (((int64_t)CALIBRATION.PRESS_7) << 4);

    // Return compensated pressure in Pa (integer)
    return (uint32_t)(p >> 8);
}

uint32_t BME_P280_BOSCH_HUM_COMPENSATE(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)CALIBRATION.HUM_4) << 20)
                  - (((int32_t)CALIBRATION.HUM_5) * v_x1_u32r)) + ((int32_t)16384)) >> 15)
                  * ((((((v_x1_u32r * ((int32_t)CALIBRATION.HUM_6)) >> 10)
                  * (((v_x1_u32r * ((int32_t)CALIBRATION.HUM_3)) >> 11) + ((int32_t)32768))) >> 10)
                  + ((int32_t)2097152)) * ((int32_t)CALIBRATION.HUM_2) + 8192)) >> 14;

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                  * ((int32_t)CALIBRATION.HUM_1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);

    return (uint32_t)(v_x1_u32r >> 12); // result in %RH * 1024
}

void BME_P280_TEMP_PRESS_HUM(void)
{
    char MESSAGE[120];
    uint8_t VALUE[8];

    I2C_READ_BYTES(BME_P280_REGISTER_PRESSURE_MSB, VALUE, 8);

    int32_t P = ((int32_t)VALUE[0] << 12) | ((int32_t)VALUE[1] << 4) | ((int32_t)VALUE[2] >> 4);
    int32_t T = ((int32_t)VALUE[3] << 12) | ((int32_t)VALUE[4] << 4) | ((int32_t)VALUE[5] >> 4);
    int32_t H = ((int32_t)VALUE[6] << 8) | VALUE[7];

    int32_t TEMP_BASIC = BME_P280_BOSCH_TEMP_COMPENSATE(T);
    uint32_t PRESSURE_PASCAL = BME_P280_BOSCH_PRESS_COMPENSATE(P);
    uint32_t HUMIDITY_BASIC = BME_P280_BOSCH_HUM_COMPENSATE(H);

    int32_t TEMP_FINAL = TEMP_BASIC / 100;
    int32_t TEMP_FRACTION = (TEMP_BASIC < 0 ? -TEMP_BASIC : TEMP_BASIC) % 100;
    uint32_t PRESSURE_HECTO = PRESSURE_PASCAL / 100;
    uint32_t PRESSURE_HECTO_FRACTION = PRESSURE_PASCAL % 100;
    uint32_t HUMIDITY_FINAL = HUMIDITY_BASIC / 1024;
    uint32_t HUMIDITY_FRACTION = (HUMIDITY_BASIC % 1024) * 100 / 1024;

    sprintf(MESSAGE, "|  %6ld.%02ld°C       |  %6lu.%02lu hPa   |   %3lu.%02lu %%RH     |\r\n",
            (long)TEMP_FINAL, (long)TEMP_FRACTION,
            (unsigned long)PRESSURE_HECTO, (unsigned long)PRESSURE_HECTO_FRACTION,
            (unsigned long)HUMIDITY_FINAL, (unsigned long)HUMIDITY_FRACTION);

    USART2_SendString(MESSAGE);
}

void SENSOR_CHECK(void)
{
    char MESSAGE[80];

    USART2_SendString("============================================================\r\n");
    USART2_SendString("[1] Sending Slave Address (0x76) and checking ACK...\r\n");
    if (!I2C_DEVICE_ALIVE()) {
        USART2_SendString("[!] ACK not received. Going idle...\r\n");
        while (1);
    }
    USART2_SendString("[*] Device ACK received...\r\n");
    delay_ms(5000);

    USART2_SendString("============================================================\r\n");
    USART2_SendString("[2] Reading chip ID register (0xD0)...\r\n");
    uint8_t id = I2C_READ_BYTE(BME_P280_REGISTER_CHIP_ID);
    sprintf(MESSAGE, "Raw value: 0x%02X\r\n", id);
    USART2_SendString(MESSAGE);
    USART2_SendString("============================================================\r\n");

    USART2_SendString("[3] Identifying the sensor (BME280 or BMP280)...\r\n");
    if (id == CHIP_ADDRESS_BME280)
        USART2_SendString("[*] BME280 detected (has humidity)\r\n");
    else if (id == CHIP_ADDRESS_BMP280)
    	USART2_SendString("[*] BMP280 detected\r\n");
    else
    {
        sprintf(MESSAGE, "[!] UNKNOWN chip ID: 0x%02X.. Going Idle...\r\n", id);
        USART2_SendString(MESSAGE);
        while (1);
    }
    USART2_SendString("============================================================\r\n");
}

int main(void)
{
    PLL_CONFIG();
    GPIO_CONFIG();
    USART2_CONFIG();
    TIM6_CONFIG();
    I2C_CONFIG();
    USART2_SendString("LAB 02 (Bare-Metal): BME/P-280 Sensor Communication by I2C\r\n");
    delay_ms(5000);
    SENSOR_CHECK();

    USART2_SendString("[4] Reading Chip Calibration Factors...\r\n");
    BME_P280_CALIBRATION_READ();
    USART2_SendString("Calibration Factors found and loaded...\r\n");
    USART2_SendString("============================================================\r\n");

    USART2_SendString("[5] Configuring BME/P280 in Normal Mode...\r\n");
    BME_P280_INIT();
    USART2_SendString("BME/P Ready for Operation...\r\n");
    delay_ms(5000);

    USART2_SendString("============================================================\r\n");
    USART2_SendString("[6] Temperature and Pressure Record per Second...\r\n");
    USART2_SendString("============================================================\r\n");
    USART2_SendString("|  Temperature (°C)  |  Pressure (hPa)  |  Humidity (%RH)  |\r\n");
    while (1) {
        BME_P280_TEMP_PRESS_HUM();
        delay_ms(1000);
    }
}
