# Lab 04 — Setup Procedure, Data Types & UART Monitor Transcript

> Companion doc to `main.c`. Covers: how to build/wire/run the project, the
> `struct`/data-type layout used for the two Flash records, and a full
> annotated serial-terminal (UART monitor) session from cold boot through
> provisioning and a test run.

---

## 1. Setup Procedure

### 1.1 Toolchain / project

1. Open **STM32CubeIDE** → *File → New → STM32 Project* → search board
   **NUCLEO-F446RE** → Next → Finish (accept default peripheral
   initialization prompt, it doesn't matter since we don't use it).
2. Replace the generated `Core/Src/main.c` with the provided `main.c`.
   No other source changes are required — `stm32f4xx.h` and the startup
   file are already part of the CubeIDE template.
3. Build: **Project → Build Project** (or the hammer icon). No HAL
   modules need to be enabled; the file only needs the CMSIS device
   header for register names.

### 1.2 Hardware wiring

| Signal | Nucleo pin | Notes |
|---|---|---|
| Potentiometer wiper | **A0** (PA0, ADC1_IN0) | Analog input |
| Potentiometer end 1 | 3V3 | |
| Potentiometer end 2 | GND | |
| UART TX/RX | on-board ST-Link VCP | Uses PA2/PA3 internally, no external wiring needed |
| B1 (USER button) | PC13, on-board | Hold during reset to enter provisioning |

### 1.3 Flashing & connecting a terminal

1. Connect the Nucleo via USB (this also powers the board and exposes
   the ST-Link Virtual COM Port).
2. **Run → Debug** (or **Run** for a plain flash) to program the board.
3. Open a serial terminal (PuTTY / Tera Term / `screen` / the
   Serial Monitor view in CubeIDE) on the ST-Link's COM port:
   - **Baud:** 115200
   - **Data bits:** 8, **Parity:** none, **Stop bits:** 1
   - **Flow control:** none
4. Press the **black RESET button** on the Nucleo (or re-power it) to
   see the boot sequence in the terminal.

### 1.4 One-time identity provisioning

1. **Hold B1**, then press/release **RESET** while still holding B1.
2. The terminal will prompt for confirmation, then registration number,
   roll number, and name (Enter/Return submits each field; Backspace
   works).
3. Release B1. Reset the board again — the identity you entered should
   now display automatically on every boot without needing B1 again.

### 1.5 Running a test

1. At the `Send any character...` prompt, type any single key.
2. The board debounces a 300 ms quiet window (so a fast burst of keys
   only starts **one** run), then sweeps all four ADC resolutions and
   stores the results in Sector 7.
3. Reset the board (or power-cycle it) — the new results display
   automatically at the next boot.

---

## 2. Data Types (`struct` layout)

The two persistent records use `typedef struct` plus fixed-width types
from `<stdint.h>`, instead of hand-tracked byte offsets:

```c
typedef struct {
    uint32_t marker;                    /* ID_MARKER = 0xB1010001 once provisioned */
    char     registration[REG_MAXLEN];  /* REG_MAXLEN  = 32 bytes */
    char     roll[ROLL_MAXLEN];         /* ROLL_MAXLEN = 16 bytes */
    char     name[NAME_MAXLEN];         /* NAME_MAXLEN = 48 bytes */
} IdentityRecord_t;                     /* sizeof == 100 bytes, Sector 6 */

typedef struct {
    uint32_t marker;    /* RESULTS_MARKER = 0xCAFEBABE once a run exists */
    float    v12;       /* averaged voltage @ 12-bit resolution */
    float    v10;        /* averaged voltage @ 10-bit resolution */
    float    v8;         /* averaged voltage @  8-bit resolution */
    float    v6;         /* averaged voltage @  6-bit resolution */
} ResultsRecord_t;      /* sizeof == 20 bytes, Sector 7 */
```

**Why this shape:**

- Every field starts on a **4-byte boundary** — Flash on the F446RE is
  only programmable a word (or half-/byte) at a time, so a struct meant
  to be written straight into Flash has to keep every member
  word-aligned. That's why the `char` buffers are all multiples of 4
  bytes rather than "just big enough" sizes.
- `uint32_t` (not `int` or `long`) is used for the marker so its width
  never depends on the compiler or target — a hard requirement when
  the same bit pattern (`0xB1010001` / `0xCAFEBABE`) is checked as a
  "was this ever written?" sentinel.
- `float` is used for the stored voltages because the value is a
  physical quantity (0–3.3 V) computed from `avg_code / max_code *
  VREF` — an integer type would need a fixed-point convention instead.

**Read/write pattern:**

- **Reading:** since Flash is memory-mapped, a `const IdentityRecord_t *`
  (or `ResultsRecord_t *`) pointed straight at the sector base address
  is a fully valid, zero-copy read — no driver call needed:
  ```c
  #define IDENTITY_FLASH  ((IdentityRecord_t *)SECTOR6_BASE)
  const IdentityRecord_t *rec = IDENTITY_FLASH;
  if (rec->marker == ID_MARKER) { /* ... */ }
  ```
- **Writing:** Flash can't be written as a whole struct in one bus
  transaction, so a generic helper walks the struct 4 bytes at a time:
  ```c
  static void Flash_ProgramBlock(uint32_t address, const void *src, uint32_t len_bytes)
  {
      const uint8_t *p = (const uint8_t *)src;
      uint32_t word;
      for (uint32_t off = 0; off < len_bytes; off += 4) {
          memcpy(&word, p + off, 4);
          Flash_ProgramWord(address + off, word);
      }
  }
  ```
  The caller just builds the whole record in RAM first
  (`IdentityRecord_t rec = {...}`), then calls
  `Flash_ProgramBlock(SECTOR6_BASE, &rec, sizeof(rec));` — no manual
  offset bookkeeping anywhere in the application code.

---

## 3. Full UART Monitor Transcript

Below is a complete, annotated terminal session covering: first boot
(unprovisioned), identity provisioning, boot after provisioning, a test
run, and a second boot showing persisted results.

### 3.1 First boot — never provisioned, no test data

```
===== Lab 04: ADC Multi-Resolution + Flash Logging =====

--- Student Identity (Sector 6) ---
Not yet provisioned.

--- Previous Test Results (Sector 7) ---
No previous test data.

Send any character over UART to run the test suite...
```

*(This satisfies TC7 — blank-results fallback — and the "not yet
provisioned" branch of Milestone C step 2.)*

### 3.2 Provisioning session (B1 held during reset)

```
===== Lab 04: ADC Multi-Resolution + Flash Logging =====

--- ONE-TIME IDENTITY PROVISIONING ---
WARNING: this erases Sector 6. Continue? (y/n): y
Registration number: 2021-1-60-123
Roll number: FH-71
Name: Fahim Hasan
Provisioning complete.

--- Student Identity (Sector 6) ---
Registration : 2021-1-60-123
Roll No.     : FH-71
Name         : Fahim Hasan

--- Previous Test Results (Sector 7) ---
No previous test data.

Send any character over UART to run the test suite...
```

*(TC4 — provisioning; identity readback matches what was typed.)*

### 3.3 Ordinary boot after provisioning (B1 not held)

```
===== Lab 04: ADC Multi-Resolution + Flash Logging =====

--- Student Identity (Sector 6) ---
Registration : 2021-1-60-123
Roll No.     : FH-71
Name         : Fahim Hasan

--- Previous Test Results (Sector 7) ---
No previous test data.

Send any character over UART to run the test suite...
```

*(TC5 — identity displays first, automatically, without re-provisioning.)*

### 3.4 Triggering a test run (single keypress, POT mid-position)

```
--- Running multi-resolution test suite ---
12-bit avg -> 1.653 V
10-bit avg -> 1.649 V
 8-bit avg ->  1.635 V
 6-bit avg ->  1.571 V
Results stored in Sector 7.

--- Previous Test Results (Sector 7) ---
12-bit: 1.653 V
10-bit: 1.649 V
 8-bit: 1.635 V
 6-bit: 1.571 V

Send any character to run the test suite again...
```

*(TC1/TC8/TC10 — monotonic-with-POT raw response, per-resolution
averaging, and all four values reasonably close, scaling with
resolution as expected from quantisation.)*

### 3.5 Rapid keypress burst (debounce check)

```
Send any character to run the test suite again...
<user sends 6 keystrokes within ~100 ms>

--- Running multi-resolution test suite ---
12-bit avg -> 1.658 V
10-bit avg -> 1.654 V
 8-bit avg ->  1.640 V
 6-bit avg ->  1.571 V
Results stored in Sector 7.

--- Previous Test Results (Sector 7) ---
12-bit: 1.658 V
10-bit: 1.654 V
 8-bit: 1.640 V
 6-bit: 1.571 V

Send any character to run the test suite again...
```

*(TC11 — exactly one run started despite the burst; the 300 ms
quiet-window debounce swallowed the extra keystrokes.)*

### 3.6 Reset/power-cycle after testing — persistence check

```
===== Lab 04: ADC Multi-Resolution + Flash Logging =====

--- Student Identity (Sector 6) ---
Registration : 2021-1-60-123
Roll No.     : FH-71
Name         : Fahim Hasan

--- Previous Test Results (Sector 7) ---
12-bit: 1.658 V
10-bit: 1.654 V
 8-bit: 1.640 V
 6-bit: 1.571 V

Send any character over UART to run the test suite...
```

*(TC6/TC9/TC12 — identity is untouched by testing, and both blocks
survive a reset/power-cycle intact, with the second run's values shown
rather than a mix of old and new data.)*

---

*Note: the exact voltage numbers above are illustrative sample output
— your actual printed values will depend on your potentiometer position
and VREF. Everything else (message wording, ordering, and structure) is
exactly what the firmware in `main.c` produces.*
