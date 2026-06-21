#include "stm32f446xx.h"
#include <stdio.h>
#include <stdint.h>
#include<string.h>

/*
 * PIN Layout Diagram in Here
 * Wiring (Arduino headers CN5 / CN6 / CN9):
 * ┌─────────────┬──────────────┬────────────┬──────────┐
 * │ BMP280 Pin  │ Nucleo Label │ Connector  │ MCU Pin  │
 * ├─────────────┼──────────────┼────────────┼──────────┤
 * │ VCC         │ 3V3          │ CN6 pin 4  │  —       │
 * │ GND         │ GND          │ CN6 pin 6  │  —       │
 * │ SCL         │ D13          │ CN5 pin 6  │  PA5     │
 * │ SDA (MOSI)  │ D11          │ CN5 pin 4  │  PA7     │
 * │ SDO (MISO)  │ D12          │ CN5 pin 5  │  PA6     │
 * │ CSB (CS)    │ D4           │ CN9 pin 5  │  PB5     │
 * └─────────────┴──────────────┴────────────┴──────────┘
 *
 * SPI1: PA5=SCK, PA6=MISO, PA7=MOSI  (AF5)
 * CS  : PB5 (GPIO output)
 */
//
//#define BME280_REG_CHIP_ID   0xD0  //for identifying the sensor, its addr
//#define BME280_REG_RESET     0xE0  //this reg used for software reset, in this addr , have to write 0xB6 for causing reset
//#define BME280_REG_CTRL_MEAS 0xF4  //measurement control register, select oversample and sensor mode
//#define BME280_REG_CONFIG    0xF5  //configuration register. for filter configuration and standby time
//#define BME280_REG_PRESS_MSB 0xF7 //begin of pressure data block . presture and temo start from 0xF7 to 0xFC
//
//#define CHIP_ID_BME280  0x60  //chip id for sensors
//#define CHIP_ID_BMP280  0x58

/* ------------------------------------------------------------------ */
/*  CS Pin: PB5 → CN9 pin 5 (D4)                                      */
/* ------------------------------------------------------------------ */
#define CS_LOW()   (GPIOB->BSRR = (1U << (5 + 16))) // resetting mainly chip select macros. spi slave is selected when cs=0
#define CS_HIGH()  (GPIOB->BSRR = (1U << 5)) //for PB5, now this is set-> high . so it deselects sensor

/* ------------------------------------------------------------------ */
/*  Calibration storage  , or calibration constants                                            */
/* ------------------------------------------------------------------ */
static uint16_t T1; // this variable -> for temperature coefficients T1
static  int16_t T2, T3; //similarly constants T2 and T3
static uint16_t P1; //this is pressure calibration coefficient P1
static  int16_t P2, P3, P4, P5;
static  int16_t P6, P7, P8, P9;
static  int32_t t_preq; //very important , for temp compensation computation. pressure compensation later uses it
// temp is always compensated before pressure

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
	 /* Enable clocks for GPIOA, GPIOB, SPI1 */
	    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
	    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //via APB2 , SPI enable

	    /* PA2/3 as USART2 TX/RX */
	    	GPIOA -> MODER &= ~((3UL << 2*2) | (3UL << 3*2)); // Reset MODE Register of GPIOA of PA2, PA3
	    	GPIOA -> MODER |= ((2UL << 2*2) | (2UL << 3*2)); // Set MODE Register to Alternate-Function Mode for PA2(TX), PA3(RX)

	    	GPIOA -> AFR[0] &= ~((0xF << 2*4) | (0xF << 3*4)); // Clearing the AFR Register Low for PA2, PA3 (each take 4 bits)
	    	GPIOA -> AFR[0] |= ((7UL << 2*4) | (7UL << 3*4)); // Set the AFR Register Low in PA2, PA3 Position to USART2 TX/RX

	    	GPIOA -> OTYPER &= ~((1UL << 2) | (1UL << 3)); // Set output type to push-pull -> 0b0
	    	GPIOA -> OSPEEDR |= ((3UL << 2*2) | (3UL << 3*2)); // Set Output Speed at very high -> 0b11
	    	GPIOA -> PUPDR &= ~((3UL << 2*2) | (3UL << 3*2)); // Set pull-up pull down to no pull-up no pull-down -> 0b00

	 /* ----- PA5=SCK, PA6=MISO, PA7=MOSI — Alternate Function 5 ---- */
	       GPIOA->MODER   |= (2U << (5*2)) | (2U << (6*2)) | (2U << (7*2)); //as alternate functions
	       GPIOA->AFR[0]  |= (5U << (5*4)) | (5U << (6*4)) | (5U << (7*4)); //AF5 for SP1 BTW
	       GPIOA->OSPEEDR |= (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)); //very high speed ensures shaper signals and better spi timing

	    /* ----- PB5 — CS (GPIO output, push-pull, high speed) so PB5 , AS CHip select---------- */
	           GPIOB->MODER   |=  (1U << (5*2));   /* output */
	           GPIOB->OTYPER  &= ~(1U << 5);       /* push-pull --> output hight / low actively */
	           GPIOB->OSPEEDR |=  (3U << (5*2));   /* high speed */
	           GPIOB->PUPDR   &= ~(3U << (5*2));   /* no pull --> np internal resistor needed */


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


/* ------------------------------------------------------------------ */
/*  SPI1 + CS (PB5)                                                    */
/* ------------------------------------------------------------------ */
static void SPI_Config(void)
{
    CS_HIGH();                           /* deselect sensor bmp280 btw , otherwise sensor will assume communaction already started */

     /* SPI1: Master, SW-NSS, CPOL=0 CPHA=0, fPCLK/32 ≈ 2.8 MHz */
    SPI1->CR1 = SPI_CR1_MSTR //enabling master mode stm-->master and bmp -->slave
              | SPI_CR1_SSM /*software slave management. Normally SPI hardware manages NSS pin. Here we manually use PB5 as CS.*/
              | SPI_CR1_SSI //internal nss signal set high. otherwise mode faulit will appera
              | (4U << SPI_CR1_BR_Pos)   /* BR[2:0]=100 → fPCLK/32 */ //--> value is 90/32=2.8125 MHz
              | SPI_CR1_SPE; //enable spi finally
}
/*
 * Since neither CPOL nor CPHA are set:
 * CPOL = 0
 *CPHA = 0
 *Clock Idle = LOW
 *Data sampled on rising edge
 *Data changed on falling edge
*/

/*Send 1 byte and receive 1 byte simultaneously. SPI is full-duplex.*/
static uint8_t SPI_Transfer(uint8_t data)
{
    while (!(SPI1->SR & SPI_SR_TXE)); //wait until buffer becomes empty
    SPI1->DR = data; //without it , spi could overwrite. write byto to dr . then spi immediately starts clk generation
    while (!(SPI1->SR & SPI_SR_RXNE)); //wait until a byte is received. Even if we only wanted to transmit: SPI always receives something.
    return (uint8_t)SPI1->DR; //like send 0xD0 , Received 0x58 , the addr of bmp  stored in D0
}

/* Single-byte read: bit7=1 → read */
static uint8_t SPI_ReadByte(uint8_t reg)
{
    CS_LOW(); //selects bmp280, slave only listens when cs is low
    SPI_Transfer(reg | 0x80); // for reading msb should be 1 anyhow
    uint8_t val = SPI_Transfer(0xFF);
    /*
     * Dummy byte.
     * Why?
     * After register address is sent:
     * BMP280 outputs data on MISO.
     * Master must continue clocking.
     * Sending:
     * 0xFF
     * creates 8 clocks.
     * Sensor returns register contents
     */
    CS_HIGH();//end of transmission
    return val; //return the value of the register
}

/* Single-byte write: bit7=0 → write */
static void SPI_WriteByte(uint8_t reg, uint8_t data)
{
    CS_LOW();
    SPI_Transfer(reg & 0x7F); //clear bit 7 . msb or bit 7 must be 0 here
    SPI_Transfer(data); //sending actual register value, in which reg have to write btw
    CS_HIGH();
}

/* Burst read: n consecutive bytes from n consecutive registers starting at reg */
static void SPI_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    CS_LOW();
    SPI_Transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI_Transfer(0xFF); //dummy byte create clks . sensor outputs next registers
    CS_HIGH();
}

/* ------------------------------------------------------------------ */
/*  Load calibration data (registers 0x88..0x9F)                      */
/* ------------------------------------------------------------------ */
static void Load_Calibration(void)
{
    uint8_t c[24]; //24 bytes actually
    SPI_ReadBurst(0x88, c, 24); //24 bytes calibration block

    T1 = (uint16_t)(c[1]  << 8 | c[0]);
    /*
     * Combine bytes.
     * Suppose:
     * c[1]=0x6B
     * c[0]=0x50
     * then:
     * 0x6B50
     * BMP280 stores LSB first.
     * Therefore:
     * High Byte <<8
     * OR Low Byte
     * Same logic for all coefficients.
     */
    T2 =  (int16_t)(c[3]  << 8 | c[2]);
    T3 =  (int16_t)(c[5]  << 8 | c[4]);
//these values are unique for every sensor
    P1 = (uint16_t)(c[7]  << 8 | c[6]);
    P2 =  (int16_t)(c[9]  << 8 | c[8]);
    P3 =  (int16_t)(c[11] << 8 | c[10]);
    P4 =  (int16_t)(c[13] << 8 | c[12]);
    P5 =  (int16_t)(c[15] << 8 | c[14]);
    P6 =  (int16_t)(c[17] << 8 | c[16]);
    P7 =  (int16_t)(c[19] << 8 | c[18]);
    P8 =  (int16_t)(c[21] << 8 | c[20]);
    P9 =  (int16_t)(c[23] << 8 | c[22]);
}

/* ------------------------------------------------------------------ */
/*  Compensation formulas — straight from datasheet (integer math)    */
/* ------------------------------------------------------------------ */

/* Returns temperature in 0.01 °C units */
//output will be temp*100 . like 2513 means 25.13 degree Celcius
static int32_t Compensate_Temperature(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)T1 << 1)))
             * ((int32_t)T2)) >> 11; //part of calibration eqtn
    var2 = (((((adc_T >> 4) - ((int32_t)T1))
             * ((adc_T >> 4) - ((int32_t)T1))) >> 12)
             * ((int32_t)T3)) >> 14;
    t_preq = var1 + var2;//VVI-> t_fine is the internal calibrated temp
    return (t_preq * 5 + 128) >> 8; //convert into 0.01 degree Celcius units
}

/* Returns pressure in Pa */
static uint32_t Compensate_Pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_preq) - 128000;
    var2 = var1 * var1 * (int64_t)P6;
    var2 = var2 + ((var1 * (int64_t)P5) << 17);
    var2 = var2 + (((int64_t)P4) << 35);
    var1 = ((var1 * var1 * (int64_t)P3) >> 8)
         + ((var1 * (int64_t)P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)P1) >> 33;
    if (var1 == 0) return 0; //prevent divide by 0
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)P7) << 4);
    return (uint32_t)p / 256;
}

/* ------------------------------------------------------------------ */
/*  Sensor init                                                        */
/* ------------------------------------------------------------------ */
static void Sensor_Init(void)
{
    SPI_WriteByte(0xE0, 0xB6);   /* soft reset, write 0xB6 to 0xE0 */
    delay_ms(10); //for reset completion
    SPI_WriteByte(0xF5, 0x10);  /* IIR filter = 2x . filter coefiicient for reducing noise*/
}

/* ------------------------------------------------------------------ */
/*  Trigger one forced-mode measurement                                */
/* ------------------------------------------------------------------ */
//spi is in sleep state . this function wakes up it
static void Trigger_Measurement(void)
{
    /* osrs_t=010(x2), osrs_p=101(x16), mode=01(forced) */
	/*
	 * Temperature oversampling x2
	 * Pressure oversampling x16
	 * Take one measurement
	 * Return to sleep
	 */
    SPI_WriteByte(0xF4, 0x57);

    /* Poll status bit[3] (measuring) until clear (~40 ms max) */
    uint32_t timeout = 200;
    while ((SPI_ReadByte(0xF3) & 0x08) && --timeout) //read status register. bit 3 ->1-measuring , 0 = finished
        delay_ms(1);
}

/* ------------------------------------------------------------------ */
/*  Read raw ADC + return compensated results                          */
/* ------------------------------------------------------------------ */
static void Read_TempPressure(int32_t *temp_c100, uint32_t *press_pa)
{
    Trigger_Measurement(); //start conversion

    /* Burst-read 0xF7..0xFC (6 bytes): press[19:0] + temp[19:0] */
    uint8_t raw[6]; //its a buffer
    SPI_ReadBurst(0xF7, raw, 6);
    /*
     * F7 Pressure MSB
     * F8 Pressure LSB
     * F9 Pressure XLSB
     * FA Temperature MSB
     * FB Temperature LSB
     * FC Temperature XLSB
     */

    int32_t adc_P = ((int32_t)raw[0] << 12)
                  | ((int32_t)raw[1] <<  4)
                  | ((int32_t)raw[2] >>  4);

    int32_t adc_T = ((int32_t)raw[3] << 12)
                  | ((int32_t)raw[4] <<  4)
                  | ((int32_t)raw[5] >>  4);
    /*
     * BMP280 stores pressure as:
     * 20 bits
     * spread across 3 bytes.
     * Example:
     * raw[0]=10101010
     * raw[1]=11001100
     * raw[2]=11110000
     * Combine into one 20-bit number
     */

    /* Temperature MUST be compensated first — sets t_fine for pressure */
    *temp_c100 = Compensate_Temperature(adc_T); //must be first before temp because t-fine is calculatef irst in here
    *press_pa  = Compensate_Pressure(adc_P);
}


int main(void)
{
    char msg[512];

    PLL_CONFIG();
    GPIO_CONFIG();
    TIM6_CONFIG();
    USART2_CONFIG();
    SPI_Config();
    delay_ms(100);

    USART2_SendString(
        "\r\n"
        "============================================================\r\n"
        "              BMP280 ENVIRONMENT MONITOR\r\n"
        "============================================================\r\n"
        " MCU        : STM32F446RE\r\n"
        " Interface  : SPI1\r\n"
        " Sensor     : BMP280/BME280\r\n"
        "============================================================\r\n"
    );

    uint8_t chip_id = SPI_ReadByte(0xD0);

    if (chip_id == 0x60)
    	USART2_SendString(" Device Detected : BME280\r\n");

    else if (chip_id == 0x58)
    	USART2_SendString(" Device Detected : BMP280\r\n");

    else
    {
        sprintf(msg,
                "\r\n"
                " ERROR: Unknown Device Detected\r\n"
                " Chip ID : 0x%02X\r\n"
                " Check SPI Connections\r\n",
                chip_id);

        USART2_SendString(msg);

        while (1);
    }

    Load_Calibration();
    Sensor_Init();

    USART2_SendString(
        " Calibration Data Loaded Successfully\r\n"
        " Sensor Initialization Complete\r\n"
        "============================================================\r\n"
        " Starting Measurements...\r\n"
        "============================================================\r\n"
    );

    while (1)
    {
        int32_t temp;
        uint32_t press;

        Read_TempPressure(&temp, &press);

        int32_t t_int  = temp / 100;
        int32_t t_frac = (temp < 0 ? -temp : temp) % 100;

        uint32_t p_int  = press / 100;
        uint32_t p_frac = press % 100;

        sprintf(msg,
                "\r\n"
                "+----------------------------------------------------------+\r\n"
                "|                  SENSOR LIVE DATA                        |\r\n"
                "+----------------------------------------------------------+\r\n"
                "| Temperature : %3ld.%02ld C                               |\r\n"
                "| Pressure    : %4lu.%02lu hPa                             |\r\n"
                "+----------------------------------------------------------+\r\n",
                (long)t_int,
                (long)t_frac,
                (unsigned long)p_int,
                (unsigned long)p_frac);

        USART2_SendString(msg);

        delay_ms(1000);
    }
}

