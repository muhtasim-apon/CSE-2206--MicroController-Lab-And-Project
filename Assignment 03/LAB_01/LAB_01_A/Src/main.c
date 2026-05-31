/*
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

#include "stm32f446xx.h"
#include <stdio.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  BMP280 Register Definitions                                        */
/* ------------------------------------------------------------------ */
#define BME280_REG_CHIP_ID   0xD0  //for identifying the sensor, its addr
#define BME280_REG_RESET     0xE0  //this reg used for software reset, in this addr , have to write 0xB6 for causing reset
#define BME280_REG_CTRL_MEAS 0xF4  //measurement control register, select oversample and sensor mode
#define BME280_REG_CONFIG    0xF5  //configuration register. for filter configuration and standby time
#define BME280_REG_PRESS_MSB 0xF7 //begin of pressure data block . presture and temo start from 0xF7 to 0xFC

#define CHIP_ID_BME280  0x60  //chip id for sensors
#define CHIP_ID_BMP280  0x58

/* ------------------------------------------------------------------ */
/*  CS Pin: PB5 → CN9 pin 5 (D4)                                      */
/* ------------------------------------------------------------------ */
#define CS_LOW()   (GPIOB->BSRR = (1U << (5 + 16))) // resetting mainly chip select macros. spi slave is selected when cs=0
#define CS_HIGH()  (GPIOB->BSRR = (1U << 5)) //for PB5, now this is set-> high . so it deselects sensor

/* ------------------------------------------------------------------ */
/*  Calibration storage  , or calibration constants                                            */
/* ------------------------------------------------------------------ */
static uint16_t dig_T1; // this variable -> for temperature coefficients T1
static  int16_t dig_T2, dig_T3; //similarly constants T2 and T3
static uint16_t dig_P1; //this is pressure calibration coefficient P1
static  int16_t dig_P2, dig_P3, dig_P4, dig_P5;
static  int16_t dig_P6, dig_P7, dig_P8, dig_P9;
static  int32_t t_fine; //very important , for temp compensation computation. pressure compensation later uses it
// temp is always compensated before pressure

static void delay_ms(uint32_t ms) //180MHz/18000=1ms
{
    for (uint32_t i = 0; i < ms * 18000; i++) __asm("NOP");
}

static void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)); //wait for clk stabilizing

    RCC->PLLCFGR = (4U   << RCC_PLLCFGR_PLLM_Pos) //8/4= 2MHz
                 | (180U << RCC_PLLCFGR_PLLN_Pos) //2*180=360 MHz
                 | (0U   << RCC_PLLCFGR_PLLP_Pos) //system clk = 360/2=180 MHz
                 | RCC_PLLCFGR_PLLSRC_HSE; //now src is the hse btw

    FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN //5 wait states. pre-fetch , instruction and data cache enable
               | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->CFGR = RCC_CFGR_HPRE_DIV1  //AHB=180/1
              | RCC_CFGR_PPRE1_DIV4 //APB1=180/4=45
              | RCC_CFGR_PPRE2_DIV2; //APB2=180/2=90

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_SW_PLL; //switch to clk src as pll
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); //switch status to pll is on till wait
}

static void UART_Config(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    GPIOA->MODER   &= ~(1U << (2*2));
    GPIOA->MODER   |= (2U << (2*2));
    GPIOA->AFR[0]  |= (7U << (2*4));
    GPIOA->OSPEEDR &= ~(3U << (2*2));
    GPIOA->OSPEEDR |= (3U << (2*2));

    USART2->BRR = (24U << 4) | 7U;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void UART_Print(const char *s)
{
    while (*s) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (uint8_t)(*s++);
    }
}

/* ------------------------------------------------------------------ */
/*  SPI1 + CS (PB5)                                                    */
/* ------------------------------------------------------------------ */
static void SPI_Config(void)
{
    /* Enable clocks for GPIOA, GPIOB, SPI1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; //via APB2 , SPI enable

    /* ----- PB5 — CS (GPIO output, push-pull, high speed) so PB5 , AS CHip select---------- */
    GPIOB->MODER   |=  (1U << (5*2));   /* output */
    GPIOB->OTYPER  &= ~(1U << 5);       /* push-pull --> output hight / low actively */
    GPIOB->OSPEEDR |=  (3U << (5*2));   /* high speed */
    GPIOB->PUPDR   &= ~(3U << (5*2));   /* no pull --> np internal resistor needed */
    CS_HIGH();                           /* deselect sensor bmp280 btw , otherwise sensor will assume communaction already started */

    /* ----- PA5=SCK, PA6=MISO, PA7=MOSI — Alternate Function 5 ---- */
    GPIOA->MODER   |= (2U << (5*2)) | (2U << (6*2)) | (2U << (7*2)); //as alternate functions
    GPIOA->AFR[0]  |= (5U << (5*4)) | (5U << (6*4)) | (5U << (7*4)); //AF5 for SP1 BTW
    GPIOA->OSPEEDR |= (3U << (5*2)) | (3U << (6*2)) | (3U << (7*2)); //very high speed ensures shaper signals and better spi timing

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

    dig_T1 = (uint16_t)(c[1]  << 8 | c[0]);
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
    dig_T2 =  (int16_t)(c[3]  << 8 | c[2]);
    dig_T3 =  (int16_t)(c[5]  << 8 | c[4]);
//these values are unique for every sensor
    dig_P1 = (uint16_t)(c[7]  << 8 | c[6]);
    dig_P2 =  (int16_t)(c[9]  << 8 | c[8]);
    dig_P3 =  (int16_t)(c[11] << 8 | c[10]);
    dig_P4 =  (int16_t)(c[13] << 8 | c[12]);
    dig_P5 =  (int16_t)(c[15] << 8 | c[14]);
    dig_P6 =  (int16_t)(c[17] << 8 | c[16]);
    dig_P7 =  (int16_t)(c[19] << 8 | c[18]);
    dig_P8 =  (int16_t)(c[21] << 8 | c[20]);
    dig_P9 =  (int16_t)(c[23] << 8 | c[22]);
}

/* ------------------------------------------------------------------ */
/*  Compensation formulas — straight from datasheet (integer math)    */
/* ------------------------------------------------------------------ */

/* Returns temperature in 0.01 °C units */
//output will be temp*100 . like 2513 means 25.13 degree Celcius
static int32_t Compensate_Temperature(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1)))
             * ((int32_t)dig_T2)) >> 11; //part of calibration eqtn
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1))
             * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12)
             * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;//VVI-> t_fine is the internal calibrated temp
    return (t_fine * 5 + 128) >> 8; //convert into 0.01 degree Celcius units
}

/* Returns pressure in Pa */
static uint32_t Compensate_Pressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8)
         + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    if (var1 == 0) return 0; //prevent divide by 0
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p / 256;
}

/* ------------------------------------------------------------------ */
/*  Sensor init                                                        */
/* ------------------------------------------------------------------ */
static void Sensor_Init(void)
{
    SPI_WriteByte(BME280_REG_RESET, 0xB6);   /* soft reset, write 0xB6 to 0xE0 */
    delay_ms(10); //for reset completion
    SPI_WriteByte(BME280_REG_CONFIG, 0x10);  /* IIR filter = 2x . filter coefiicient for reducing noise*/
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
    SPI_WriteByte(BME280_REG_CTRL_MEAS, 0x57);

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
    SPI_ReadBurst(BME280_REG_PRESS_MSB, raw, 6);
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

//int main(void)
//{
//    char msg[80];
//
//    SystemClock_Config();
//    UART_Config();
//    SPI_Config();
//    delay_ms(100);
//
//    /* Detect sensor */
//    uint8_t chip_id = SPI_ReadByte(BME280_REG_CHIP_ID);
//    if (chip_id == CHIP_ID_BME280)
//        UART_Print("BME280 detected.\r\n");
//    else if (chip_id == CHIP_ID_BMP280)
//        UART_Print("BMP280 detected.\r\n");
//    else {
//        sprintf(msg, "Unknown chip ID: 0x%02X — check wiring!\r\n", chip_id);
//        UART_Print(msg);
//        while (1);
//    }
//
//    Load_Calibration();
//    Sensor_Init();
//    UART_Print("Calibration loaded. Starting readings...\r\n\r\n");
//
//    while (1)
//    {
//        int32_t  temp;
//        uint32_t press;
//
//        Read_TempPressure(&temp, &press);
//
//        /* temp in 0.01 °C → split integer + fraction */
//        int32_t t_int  =  temp / 100;
//        int32_t t_frac = (temp < 0 ? -temp : temp) % 100;
//
//        /* pressure in Pa → hPa with 2 decimal places */
//        uint32_t p_int  = press / 100;
//        uint32_t p_frac = press % 100;
//
//        sprintf(msg, "Temp: %ld.%02ld C   Pressure: %lu.%02lu hPa\r\n",
//                (long)t_int, (long)t_frac,
//                (unsigned long)p_int, (unsigned long)p_frac);
//        UART_Print(msg);
//
//        delay_ms(1000);
//    }
//}
int main(void)
{
    char msg[256];

    SystemClock_Config();
    UART_Config();
    SPI_Config();
    delay_ms(100);

    UART_Print(
        "\r\n"
        "============================================================\r\n"
        "              BMP280 ENVIRONMENT MONITOR\r\n"
        "============================================================\r\n"
        " MCU        : STM32F446RE\r\n"
        " Interface  : SPI1\r\n"
        " Sensor     : BMP280/BME280\r\n"
        "============================================================\r\n"
    );

    uint8_t chip_id = SPI_ReadByte(BME280_REG_CHIP_ID);

    if (chip_id == CHIP_ID_BME280)
        UART_Print(" Device Detected : BME280\r\n");

    else if (chip_id == CHIP_ID_BMP280)
        UART_Print(" Device Detected : BMP280\r\n");

    else
    {
        sprintf(msg,
                "\r\n"
                " ERROR: Unknown Device Detected\r\n"
                " Chip ID : 0x%02X\r\n"
                " Check SPI Connections\r\n",
                chip_id);

        UART_Print(msg);

        while (1);
    }

    Load_Calibration();
    Sensor_Init();

    UART_Print(
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

        UART_Print(msg);

        delay_ms(1000);
    }
}
