/**
 * segled.h - STM8S 2-Digit 7-Segment LED Driver (Object-Oriented, Common-Anode)
 *
 * Drives a 2-digit common-ANODE 8-segment (a..g + dp) LED display using
 * 10 GPIO pins (8 segment + 2 digit select) with software dynamic
 * scanning. Each module exposes a SEGLED_t abstraction with method
 * binding, mirroring the OO style of project2/project3/project4.
 *
 * Difference vs project4 (4-digit common-CATHODE):
 *   - digits reduced to 2 (digits[2], 2 digit-select pins)
 *   - common-ANODE wiring: segment pins drive the shared ANODEs, so a
 *     segment lights with ACTIVE-LOW output (segment code table is the
 *     bitwise complement of the common-cathode table);
 *   - digit pins drive the shared CATHODEs through NPN / directly, and
 *     a digit is selected with ACTIVE-HIGH (default digit state LOW).
 *
 * Pin mapping (TSSOP20, common-anode, active-high digit select):
 *   SEG_A PA1   SEG_B PC5   SEG_C PA3   SEG_D PC3
 *   SEG_E PC4   SEG_F PA2   SEG_G PC6   SEG_DP PC7
 *   DIG1 PD2   DIG2 PD3
 * (all 10 pins deliberately avoid HC-SR04 PB5/PB4,
 *  UART PD5/PD6, BEEP PD4 and SWIM PD1; PD0 is not brought out on
 *  TSSOP20)
 *
 * Digit value encoding (stored in digits[]):
 *   0..9  -> digit, 10..15 -> A..F, 16 -> blank, 17 -> '-', 18 -> dp-only
 *   SEG_FLAG_DP (0x20) ORed with a digit 0..15 -> that digit with the
 *   decimal point lit.
 *
 * Usage:
 *   #include "segled.h"
 *   SEGLED_t seg = SEGLED_Create();          // default pin map above
 *   seg.init(&seg);
 *   seg.setDigit(&seg, 0, 1);                // show "1" on digit 0
 *   seg.show(&seg, "42");                    // convenience (2 chars)
 *   seg.onTime(&seg, tick);                  // call every 1ms (TIM1 ISR)
 */

#ifndef SEGLED_H
#define SEGLED_H

#include <stdint.h>

/* ========== GPIO Register Offset Macros ==========
 * STM8S GPIO registers layout per port (same as led.h in project2):
 *   +0: ODR    Output Data Register
 *   +1: IDR    Input Data Register
 *   +2: DDR    Data Direction Register
 *   +3: CR1    Control Register 1
 *   +4: CR2    Control Register 2
 */
#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_ODR(port)          GPIO_REG(port, 0)
#define GPIO_IDR(port)          GPIO_REG(port, 1)
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* ========== 7-Segment Display Values ==========
 * Stored per digit, translated to segment codes by segled.c:
 *   0..9  -> digit, 10..15 -> A..F, 16 -> blank, 17 -> '-'
 */
#define SEG_DIGIT_0      0
#define SEG_DIGIT_BLANK  16
#define SEG_DIGIT_MINUS  17
#define SEG_DIGIT_DOT    18   /* dp-only */
#define SEG_FLAG_DP      0x20 /* OR with 0..15 -> digit with dp lit */

#define SEG_DIGIT_COUNT  2    /* 2-digit display */

/* ========== 2-Digit LED Class Definition ========== */

typedef struct SEGLED SEGLED_t;

struct SEGLED {
    uint16_t seg_port[8];       /* GPIO base per segment a..g,dp */
    uint8_t  seg_pin[8];        /* pin number per segment */
    uint8_t  seg_mask[8];       /* precomputed bit masks */
    uint16_t dig_port[SEG_DIGIT_COUNT]; /* GPIO base per digit select */
    uint8_t  dig_pin[SEG_DIGIT_COUNT];  /* pin number per digit */
    uint8_t  dig_mask[SEG_DIGIT_COUNT]; /* precomputed bit masks */

    /* --- Display state --- */
    uint8_t  digits[SEG_DIGIT_COUNT];   /* values 0..9 / A..F / 16=blank / 17='-' / dp flag */
    uint8_t  cur_digit;         /* digit currently being scanned (0..1) */

    /* --- Public methods --- */
    void (*init)   (SEGLED_t *self);                     /* configure 10 GPIO outputs */
    void (*setDigit)(SEGLED_t *self, uint8_t pos, uint8_t value); /* update one digit buffer */
    void (*show)   (SEGLED_t *self, const char *str);    /* set 2 digits from string */
    void (*onTime) (SEGLED_t *self, uint16_t tick);      /* 1ms dynamic scan (TIM1 ISR) */
};

/* ========== Constructor ========== */

/**
 * Create a 2-digit common-anode LED object with the default pin map:
 *   segments PA1 PC5 PA3 PC3 PC4 PA2 PC6 PC7 ; digits PD2 PD3.
 * Uses static storage — safe for single-instance use on STM8S103.
 * The constructor does NOT configure the GPIOs; call init() first.
 *
 * @return  fully initialized SEGLED_t object
 */
SEGLED_t SEGLED_Create(void);

#endif /* SEGLED_H */
