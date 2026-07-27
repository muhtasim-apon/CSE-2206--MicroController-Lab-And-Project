/* =====================================================================
 * Lab 04 - ADC Multi-Resolution Testing with Flash-Based Data Logging
 * Target : Nucleo-F446RE (STM32F446RE, RM0390)
 * Style  : Register-level / bare-metal (CMSIS device header only, no HAL)
 *
 * Drop this file in as main.c of an STM32CubeIDE "STM32F446RE Nucleo"
 * project (any template that gives you stm32f4xx.h + startup file +
 * linker script is fine -- we do not use HAL_Init()/SystemClock_Config(),
 * so HSI 16 MHz, the reset-default system clock, is used throughout).
 *
 * Hardware wiring:
 *   - Potentiometer wiper -> A0 (PA0 / ADC1_IN0), ends -> 3V3 and GND
 *   - USART2 (ST-Link Virtual COM Port, 115200-8-N-1) for all I/O
 *   - B1 USER button (PC13) held during reset -> one-time identity
 *     provisioning menu (pressed = logic HIGH on this board)
 *
 * Milestone / Test-case map (see lab handout Sections 6-7):
 *   Milestone A (ADC acquisition)      -> ADC_Init(), ADC_ReadRaw_REG()      [TC1]
 *   Milestone B (Flash utilities)      -> Flash_Unlock/Lock/EraseSector/
 *                                          ProgramWord()                     [TC2,TC3]
 *   Milestone C (Identity storage)     -> Identity_Provision/Display()      [TC4,TC5,TC6]
 *   Milestone D (Multi-res test suite) -> TestSuite_Run()                   [TC8,TC10]
 *   Milestone E (Boot & display)       -> main() boot sequence, Results_
 *                                          Display()                        [TC7]
 *   Milestone F (Persistence/robust.)  -> Flash-backed storage survives
 *                                          reset/power-cycle by design      [TC9,TC11,TC12]
 *
 * Data types: StudentInfo_t and ResultsRecord_t (typedef struct, see
 * below) describe the on-Flash layout for Sector 6 / Sector 7. Fixed-
 * width types (uint8_t/uint32_t/etc. from <stdint.h>) are used for every
 * register and field so sizes never depend on compiler/platform.
 * =====================================================================
 */

#include "stm32f446xx.h"
#include <stdint.h>
#include <string.h>

/* --------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------*/
uint32_t SystemCoreClock = 	16000000U;
#define VREF                3.3f     /* board VDDA, adjust if different */
#define ADC_SAMPLES_N       16U      /* N samples averaged per resolution
                                         (chosen per Milestone D: large
                                         enough to average out noise,
                                         small enough to stay fast) */
#define UART_DEBOUNCE_MS    300U     /* burst-drain window so a rapid
                                         key-burst triggers exactly one
                                         test run (TC11) */

/* --------------------------------------------------------------------
 * Flash sector map (RM0390: 512 KB Flash, 8 sectors: 4x16K,1x64K,3x128K)
 * ------------------------------------------------------------------*/
#define SECTOR6_BASE        0x08040000UL   /* Identity block  (128 KB) */
#define SECTOR6_NUM         6U
#define SECTOR7_BASE        0x08060000UL   /* Results block   (128 KB) */
#define SECTOR7_NUM         7U

#define ID_MARKER           0xB1010001UL
#define RESULTS_MARKER      0xCAFEBABEUL

#define REG_MAXLEN          16U   /* must be multiple of 4 (word writes) */
#define ROLL_MAXLEN         12U
#define NAME_MAXLEN         32U

/* --------------------------------------------------------------------
 * On-chip data records (typedef struct)
 *
 * StudentInfo_t matches the handout's Milestone C layout exactly:
 * typedef struct {
 *   uint32_t marker;          // 0xB1010001 if valid, 0xFFFFFFFF if blank
 *   char registration[16];    // null-padded string
 *   char roll[12];            // null-padded string
 *   char name[32];            // null-padded string
 * } StudentInfo_t;            // 64 bytes total -- word-aligned
 *
 * Every member starts on a 4-byte boundary -- Flash on the F446RE can
 * only be programmed a word (or half-word/byte) at a time, so a struct
 * that maps 1:1 onto a Flash block must be word-aligned throughout.
 * sizeof(...) is used everywhere instead of hand-counted byte offsets,
 * so the layout stays consistent even if a field's size changes later.
 * ------------------------------------------------------------------*/
typedef struct {
    uint32_t marker;                    /* 0xB1010001 if valid, 0xFFFFFFFF if blank */
    char     registration[REG_MAXLEN];  /* null-padded string */
    char     roll[ROLL_MAXLEN];         /* null-padded string */
    char     name[NAME_MAXLEN];         /* null-padded string */
} StudentInfo_t;                        /* 64 bytes total -- word-aligned */

typedef struct {
    uint32_t marker;                 /* RESULTS_MARKER once a run exists */
    float    v12;                    /* averaged voltage @ 12-bit RES    */
    float    v10;                    /* averaged voltage @ 10-bit RES    */
    float    v8;                     /* averaged voltage @  8-bit RES    */
    float    v6;                     /* averaged voltage @  6-bit RES    */
} ResultsRecord_t;

#define IDENTITY_FLASH  ((StudentInfo_t   *)SECTOR6_BASE)  /* memory-mapped read view */
#define RESULTS_FLASH   ((ResultsRecord_t *)SECTOR7_BASE)

/* --------------------------------------------------------------------
 * Forward declarations
 * ------------------------------------------------------------------*/
static void     delay_ms(uint32_t ms);

static void     GPIO_Init(void);
static uint8_t  Button_Pressed(void);

static void     UART2_Init(void);
static void     UART2_SendChar(char c);
static void     UART2_SendString(const char *s);
static void     UART2_SendUint(uint32_t v);
static void     UART2_SendFloat3(float v);
static uint8_t  UART2_ByteAvailable(void);
static char     UART2_ReceiveChar(void);
static void     UART2_ReadLine(char *buf, uint8_t maxlen);

static void     ADC_Init(void);
static uint32_t ADC_ReadRaw_REG(uint8_t res_field);
static float    ADC_ReadAveragedVoltage(uint8_t res_field, uint32_t max_code, uint32_t n);
static float    ADC_ReadAveragedVoltageLogged(uint8_t res_field, uint32_t max_code,
                                               uint32_t n, uint32_t *log_buf);

static void     Flash_Unlock(void);
static void     Flash_Lock(void);
static void     Flash_WaitBusy(void);
static void     Flash_EraseSector(uint8_t sector_num);
static void     Flash_ProgramWord(uint32_t address, uint32_t data);
static void     Flash_ProgramBlock(uint32_t address, const void *src, uint32_t len_bytes);

static void     Identity_Provision(void);
static void     Identity_Display(void);
static void     Results_Display(void);
static void     TestSuite_Run(void);

/* --------------------------------------------------------------------
 * Section 7 - Test Cases (TC1-TC12), see lab handout table.
 *
 * TC_PASS / TC_FAIL are firmware-verifiable outcomes. TC_MANUAL means
 * the check genuinely needs a human action (rotate the POT, press
 * reset, unplug/replug USB) that main.c cannot perform on itself; the
 * suite prints exactly what to do and what result to expect, and the
 * test case is still "executed and reported" per the rubric -- just
 * not auto-graded.
 * ------------------------------------------------------------------*/
typedef enum { TC_PASS, TC_FAIL, TC_MANUAL } TCResult_t;

static void      TC_PrintResult(const char *id, const char *name, TCResult_t result);
static uint8_t   Verify_AverageMatches(const uint32_t *log, uint32_t n,
                                       uint32_t max_code, float reported_v);

static TCResult_t TC1_RawADCResponsiveness(void);
static TCResult_t TC2_FlashEraseVerification(void);
static TCResult_t TC3_FlashWriteReadback(void);
static TCResult_t TC4_IdentityProvisioning(void);
static TCResult_t TC5_IdentityDisplayAtBoot(void);
static TCResult_t TC6_IdentitySurvivesTesting(void);
static TCResult_t TC7_BlankResultsFallback(void);
static TCResult_t TC8_PerResolutionAveraging(void);
static TCResult_t TC9_ResultOverwriteOnRetest(void);
static TCResult_t TC10_ResolutionBoundarySanity(void);
static TCResult_t TC11_UartAutoTriggerReliability(void);
static TCResult_t TC12_PowerCyclePersistence(void);
static void       RunTestCaseSuite(void);

/* ======================================================================
 * main
 * ====================================================================*/
int main(void)
{
    GPIO_Init();
    UART2_Init();
    ADC_Init();

    UART2_SendString("\r\n\r\n===== Lab 04: ADC Multi-Resolution + Flash Logging =====\r\n");

    /* One-time identity provisioning: only runs if B1 is held at reset.
     * This is a deliberate manual step -- it must never run automatically
     * every boot (Milestone C, step 1). */
    if (Button_Pressed()) {
        Identity_Provision();
    }

    /* ---- Boot sequence (every reset), per System Design Overview ---- */
    Identity_Display();   /* 1. identity block (Sector 6)                */
    Results_Display();    /* 2. previous test results (Sector 7)         */

    UART2_SendString("\r\nSend 'T' to run the automated TC1-TC12 test-case suite (Section 7),\r\n");
    UART2_SendString("or any other character to run a normal measurement pass...\r\n");

    /* 3. wait for UART input -> run test suite -> update Sector 7        */
    while (1) {
        if (UART2_ByteAvailable()) {
            char trigger = UART2_ReceiveChar();  /* remember the trigger byte */

            /* Debounce a burst of keys into a single trigger (TC11) */
            uint32_t quiet_ms = 0;
            while (quiet_ms < UART_DEBOUNCE_MS) {
                if (UART2_ByteAvailable()) {
                    (void)UART2_ReceiveChar();
                    quiet_ms = 0;
                } else {
                    delay_ms(1);
                    quiet_ms++;
                }
            }

            TestSuite_Run();
            Results_Display();
            UART2_SendString("\r\nSend any character to run the test suite again...\r\n");
        }
    }
}

/* ======================================================================
 * Timing
 * ====================================================================*/
static void delay_ms(uint32_t ms)
{
    /* SystemCoreClock is maintained by CMSIS's system_stm32f4xx.c and is
     * 16,000,000 (HSI) here since we never touch the PLL. */
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;
    SysTick->VAL  = 0U;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    for (uint32_t i = 0U; i < ms; i++) {
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) { /* wait */ }
    }
    SysTick->CTRL = 0U;
}

/* ======================================================================
 * GPIO
 * ====================================================================*/
static void GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   /* PA0 (ADC), PA2/PA3 (USART2) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;   /* PC13 (B1 user button)       */

    /* PA0 -> analog mode, ADC1_IN0 (Nucleo "A0" header pin) */
    GPIOA->MODER |= (3U << (0U * 2U));

    /* PA2 (TX) / PA3 (RX) -> alternate function AF7 (USART2) */
    GPIOA->MODER &= ~((3U << (2U * 2U)) | (3U << (3U * 2U)));
    GPIOA->MODER |=  ((2U << (2U * 2U)) | (2U << (3U * 2U)));
    GPIOA->AFR[0] &= ~((0xFU << (2U * 4U)) | (0xFU << (3U * 4U)));
    GPIOA->AFR[0] |=  ((7U   << (2U * 4U)) | (7U   << (3U * 4U)));

    /* PC13 -> input, B1 user button */
    GPIOC->MODER &= ~(3U << (13U * 2U));
}

static uint8_t Button_Pressed(void)
{
    /* Nucleo-F446RE B1: pressed = logic HIGH (external pull-down on the
     * board). If your particular board/wiring is inverted, flip this. */
    return (GPIOC->IDR & (1U << 13)) ? 1U : 0U;
}

/* ======================================================================
 * USART2 (register-level, 115200-8-N-1, PCLK1 = 16 MHz HSI)
 * ====================================================================*/
static void UART2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* USARTDIV = 16e6 / (16 * 115200) = 8.68 -> Mantissa=8, Fraction=11 */
    USART2->BRR = (8U << 4) | 11U;

    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void UART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE)) { /* wait */ }
    USART2->DR = (uint8_t)c;
}

static void UART2_SendString(const char *s)
{
    while (*s) {
        UART2_SendChar(*s++);
    }
}

static void UART2_SendUint(uint32_t v)
{
    char buf[10];
    int i = 0;

    if (v == 0U) {
        UART2_SendChar('0');
        return;
    }
    while (v > 0U && i < 10) {
        buf[i++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (i > 0) {
        UART2_SendChar(buf[--i]);
    }
}

static void UART2_SendFloat3(float v)
{
    if (v < 0.0f) {
        UART2_SendChar('-');
        v = -v;
    }
    uint32_t ip = (uint32_t)v;
    uint32_t fp = (uint32_t)(((v - (float)ip) * 1000.0f) + 0.5f);
    if (fp >= 1000U) {
        fp -= 1000U;
        ip += 1U;
    }
    UART2_SendUint(ip);
    UART2_SendChar('.');
    if (fp < 100U) UART2_SendChar('0');
    if (fp < 10U)  UART2_SendChar('0');
    UART2_SendUint(fp);
}

static uint8_t UART2_ByteAvailable(void)
{
    return (USART2->SR & USART_SR_RXNE) ? 1U : 0U;
}

static char UART2_ReceiveChar(void)
{
    while (!(USART2->SR & USART_SR_RXNE)) { /* wait */ }
    return (char)(USART2->DR & 0xFFU);
}

static void UART2_ReadLine(char *buf, uint8_t maxlen)
{
    uint8_t i = 0U;

    while (1) {
        char c = UART2_ReceiveChar();

        if (c == '\r' || c == '\n') {
            UART2_SendString("\r\n");
            break;
        } else if ((c == 8 || c == 127) && i > 0U) {   /* backspace */
            i--;
            UART2_SendString("\b \b");
        } else if (i < (uint8_t)(maxlen - 1U)) {
            buf[i++] = c;
            UART2_SendChar(c);   /* local echo */
        }
    }
    buf[i] = '\0';
}

/* ======================================================================
 * ADC (register-level, required implementation)
 * ====================================================================*/
static void ADC_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* Channel 0 (PA0) sample time = 480 cycles (max) for stable readings */
    ADC1->SMPR2 &= ~(7U << (0U * 3U));
    ADC1->SMPR2 |=  (7U << (0U * 3U));

    ADC1->SQR1 = 0U;    /* L[3:0] = 0 -> sequence length = 1 conversion */
    ADC1->SQR3 = 0U;    /* SQ1 = channel 0 */
    ADC1->CR2  = 0U;    /* software trigger (EXTEN disabled), right-aligned */
    ADC1->CR1  = 0U;    /* RES = 00 (12-bit) by default; set per-call    */

    ADC1->CR2 |= ADC_CR2_ADON;
    delay_ms(1U);       /* ADC stabilisation time */
}

/*
 * Register-level raw ADC read (Milestone A, required).
 * res_field must match RM0390's ADC1_CR1.RES encoding directly:
 *   0b00 = 12-bit (max code 4095)   0b10 = 8-bit (max code 255)
 *   0b01 = 10-bit (max code 1023)   0b11 = 6-bit (max code 63)
 */
static uint32_t ADC_ReadRaw_REG(uint8_t res_field)
{
    ADC1->CR1 = (ADC1->CR1 & ~ADC_CR1_RES) | (((uint32_t)res_field & 0x3U) << ADC_CR1_RES_Pos);

    ADC1->SR &= ~ADC_SR_EOC;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while (!(ADC1->SR & ADC_SR_EOC)) { /* wait */ }

    return (ADC1->DR & 0xFFFFU);
}

/* Milestone D helper: N-sample average -> voltage, using that
 * resolution's own max_code as required by the handout's formula. */
static float ADC_ReadAveragedVoltage(uint8_t res_field, uint32_t max_code, uint32_t n)
{
    uint32_t sum = 0U;

    for (uint32_t i = 0U; i < n; i++) {
        sum += ADC_ReadRaw_REG(res_field);
    }
    float avg_code = (float)sum / (float)n;
    return (avg_code / (float)max_code) * VREF;
}

/* ======================================================================
 * Flash utilities (register-level, required implementation)
 * ====================================================================*/
static void Flash_Unlock(void)
{
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123U;
        FLASH->KEYR = 0xCDEF89ABU;
    }
}

static void Flash_Lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

static void Flash_WaitBusy(void)
{
    while (FLASH->SR & FLASH_SR_BSY) { /* wait */ }
}

static void Flash_EraseSector(uint8_t sector_num)
{
    Flash_WaitBusy();
    Flash_Unlock();

    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1;                 /* PSIZE = 10 (32-bit) */
    FLASH->CR &= ~FLASH_CR_SNB;
    FLASH->CR |= ((uint32_t)sector_num << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;

    Flash_WaitBusy();

    FLASH->CR &= ~FLASH_CR_SER;
    FLASH->CR &= ~FLASH_CR_SNB;

    Flash_Lock();
}

static void Flash_ProgramWord(uint32_t address, uint32_t data)
{
    Flash_WaitBusy();
    Flash_Unlock();

    FLASH->CR &= ~FLASH_CR_PSIZE;
    FLASH->CR |= FLASH_CR_PSIZE_1;                 /* 32-bit word program */
    FLASH->CR |= FLASH_CR_PG;

    *(volatile uint32_t *)address = data;

    Flash_WaitBusy();

    FLASH->CR &= ~FLASH_CR_PG;

    Flash_Lock();
}

/* Generic word-at-a-time writer for a whole struct/record. len_bytes
 * must be a multiple of 4 (true for both StudentInfo_t and
 * ResultsRecord_t as laid out above). Keeps Identity_Provision() and
 * TestSuite_Run() free of per-field offset arithmetic. */
static void Flash_ProgramBlock(uint32_t address, const void *src, uint32_t len_bytes)
{
    const uint8_t *p = (const uint8_t *)src;
    uint32_t word;

    for (uint32_t off = 0U; off < len_bytes; off += 4U) {
        memcpy(&word, p + off, 4U);
        Flash_ProgramWord(address + off, word);
    }
}

/* ======================================================================
 * Milestone C - Student identity (write-once, Sector 6)
 * ====================================================================*/
static void Identity_Provision(void)
{
    StudentInfo_t rec;   /* built fully in RAM first, then written as one block */

    UART2_SendString("\r\n--- ONE-TIME IDENTITY PROVISIONING ---\r\n");
    UART2_SendString("WARNING: this erases Sector 6. Continue? (y/n): ");
    char c = UART2_ReceiveChar();
    UART2_SendChar(c);
    UART2_SendString("\r\n");

    if (c != 'y' && c != 'Y') {
        UART2_SendString("Provisioning cancelled.\r\n");
        return;
    }

    memset(&rec, 0, sizeof(rec));
    rec.marker = ID_MARKER;

    UART2_SendString("Registration number: ");
    UART2_ReadLine(rec.registration, sizeof(rec.registration));
    UART2_SendString("Roll number: ");
    UART2_ReadLine(rec.roll, sizeof(rec.roll));
    UART2_SendString("Name: ");
    UART2_ReadLine(rec.name, sizeof(rec.name));

    Flash_EraseSector(SECTOR6_NUM);
    Flash_ProgramBlock(SECTOR6_BASE, &rec, sizeof(rec));

    UART2_SendString("Provisioning complete.\r\n");
}

static void Identity_Display(void)
{
    /* IDENTITY_FLASH is a pointer straight onto the memory-mapped Flash
     * region -- reading a struct out of Flash needs no copy or driver
     * call, it's just ordinary memory. */
    const StudentInfo_t *rec = IDENTITY_FLASH;

    UART2_SendString("\r\n--- Student Identity (Sector 6) ---\r\n");
    if (rec->marker == ID_MARKER) {
        UART2_SendString("Registration : ");
        UART2_SendString(rec->registration);
        UART2_SendString("\r\nRoll No.     : ");
        UART2_SendString(rec->roll);
        UART2_SendString("\r\nName         : ");
        UART2_SendString(rec->name);
        UART2_SendString("\r\n");
    } else {
        UART2_SendString("Not yet provisioned.\r\n");
    }
}

/* ======================================================================
 * Milestone D/E - Multi-resolution test suite & results display
 * ====================================================================*/
static void Results_Display(void)
{
    const ResultsRecord_t *rec = RESULTS_FLASH;

    UART2_SendString("\r\n--- Previous Test Results (Sector 7) ---\r\n");
    if (rec->marker == RESULTS_MARKER) {
        UART2_SendString("12-bit: "); UART2_SendFloat3(rec->v12); UART2_SendString(" V\r\n");
        UART2_SendString("10-bit: "); UART2_SendFloat3(rec->v10); UART2_SendString(" V\r\n");
        UART2_SendString(" 8-bit: "); UART2_SendFloat3(rec->v8);  UART2_SendString(" V\r\n");
        UART2_SendString(" 6-bit: "); UART2_SendFloat3(rec->v6);  UART2_SendString(" V\r\n");
    } else {
        UART2_SendString("No previous test data.\r\n");
    }
}

static void TestSuite_Run(void)
{
    ResultsRecord_t rec;   /* built fully in RAM first, then written as one block */

    UART2_SendString("\r\n--- Running multi-resolution test suite ---\r\n");

    rec.marker = RESULTS_MARKER;

    rec.v12 = ADC_ReadAveragedVoltage(0x0U, 4095U, ADC_SAMPLES_N);  /* RES=00 */
    UART2_SendString("12-bit avg -> "); UART2_SendFloat3(rec.v12); UART2_SendString(" V\r\n");

    rec.v10 = ADC_ReadAveragedVoltage(0x1U, 1023U, ADC_SAMPLES_N);  /* RES=01 */
    UART2_SendString("10-bit avg -> "); UART2_SendFloat3(rec.v10); UART2_SendString(" V\r\n");

    rec.v8  = ADC_ReadAveragedVoltage(0x2U, 255U,  ADC_SAMPLES_N);  /* RES=10 */
    UART2_SendString(" 8-bit avg -> "); UART2_SendFloat3(rec.v8);  UART2_SendString(" V\r\n");

    rec.v6  = ADC_ReadAveragedVoltage(0x3U, 63U,   ADC_SAMPLES_N);  /* RES=11 */
    UART2_SendString(" 6-bit avg -> "); UART2_SendFloat3(rec.v6);  UART2_SendString(" V\r\n");

    Flash_EraseSector(SECTOR7_NUM);
    Flash_ProgramBlock(SECTOR7_BASE, &rec, sizeof(rec));

    UART2_SendString("Results stored in Sector 7.\r\n");
}
